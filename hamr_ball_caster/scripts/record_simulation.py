#!/usr/bin/env python3
"""Encode Gazebo camera images into MP4 using their simulation timestamps.

Run alongside a world containing worlds/waypoint_camera.sdf and the Sensors
system. Send SIGINT after the trajectory finishes. The recorder owns its image
bridge and ffmpeg process, and writes frame/timing evidence beside the video.
"""

import argparse
from fractions import Fraction
import json
import math
import os
from pathlib import Path
import queue
import shutil
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image


def stop_process(process):
    if process and process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--ready-file", type=Path)
    parser.add_argument("--metadata", type=Path)
    parser.add_argument("--topic", default="/hamr/recording_camera/image")
    parser.add_argument("--fps", type=int, default=25)
    parser.add_argument("--duration", type=float, default=0,
                        help="Stop after this many simulation seconds; 0 waits for SIGINT")
    parser.add_argument("--wall-timeout", type=float, default=3600)
    parser.add_argument("--no-bridge", action="store_true",
                        help="Use an image bridge already running in this ROS domain")
    args = parser.parse_args()
    if args.fps < 1 or args.fps > 120 or args.duration < 0 or args.wall_timeout <= 0:
        parser.error("fps must be 1..120, duration >= 0, wall-timeout > 0")
    if not shutil.which("ffmpeg"):
        parser.error("ffmpeg is required; source scripts/env.sh or install ffmpeg")
    output = args.output.expanduser().resolve()
    if output.exists():
        parser.error(f"Refusing to overwrite existing video: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    metadata = (args.metadata or output.with_suffix(output.suffix + ".json")).resolve()
    if args.ready_file:
        args.ready_file.parent.mkdir(parents=True, exist_ok=True)
        args.ready_file.unlink(missing_ok=True)

    stopping = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stopping.set())
    pending = queue.Queue(maxsize=60)
    stats = {
        "output": str(output), "topic": args.topic, "fps": args.fps,
        "timing": "camera message simulation timestamps, constant frame rate",
        "frames_received": 0, "frames_encoded": 0, "frames_duplicated": 0,
        "frames_same_timestamp_bin": 0, "frames_queue_dropped": 0,
        "max_image_gap_s": 0.0,
        # Actual fresh frames written to ffmpeg, excluding timeline duplicates.
        # Enables motion-specific playback-gap checks after the route finishes.
        "encoded_source_timestamps_s": [],
    }
    errors = []
    encoder = None
    bridge = None
    worker = None
    first_stamp = None
    last_stamp = None
    started = time.monotonic()
    encoder_log_path = output.with_suffix(output.suffix + ".ffmpeg.log")
    bridge_log_path = output.with_suffix(output.suffix + ".bridge.log")
    encoder_log = encoder_log_path.open("w")
    bridge_log = bridge_log_path.open("w")

    def encode_frames():
        previous = None
        index = -1
        origin = None
        try:
            while True:
                item = pending.get()
                if item is None:
                    break
                stamp, pixels = item
                if origin is None:
                    origin = stamp
                target_index = round((stamp - origin) * args.fps)
                if target_index <= index:
                    stats["frames_same_timestamp_bin"] += 1
                    continue
                if target_index - index > args.fps * 60:
                    raise RuntimeError("Camera time jumped more than 60 s; refusing ambiguous replay")
                while index + 1 < target_index:
                    encoder.stdin.write(previous)
                    stats["frames_encoded"] += 1
                    stats["frames_duplicated"] += 1
                    index += 1
                encoder.stdin.write(pixels)
                stats["frames_encoded"] += 1
                stats["encoded_source_timestamps_s"].append(stamp)
                index += 1
                previous = pixels
        except Exception as exc:
            errors.append(f"encoder: {exc}")
            stopping.set()

    def on_image(message):
        nonlocal encoder, worker, first_stamp, last_stamp
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if not math.isfinite(stamp):
            errors.append("Non-finite camera timestamp")
            stopping.set()
            return
        if message.encoding not in ("rgb8", "bgr8"):
            errors.append(f"Unsupported camera encoding {message.encoding!r}; use R8G8B8")
            stopping.set()
            return
        if message.width % 2 or message.height % 2:
            errors.append("H.264 yuv420p requires even image width and height")
            stopping.set()
            return
        row_bytes = message.width * 3
        if message.step < row_bytes or len(message.data) < message.step * message.height:
            errors.append("Camera image data/row stride is inconsistent")
            stopping.set()
            return
        if encoder is None:
            stats.update(width=message.width, height=message.height,
                         encoding=message.encoding, first_sim_time_s=stamp)
            first_stamp = stamp
            encoder = subprocess.Popen([
                "ffmpeg", "-nostdin", "-hide_banner", "-loglevel", "warning",
                "-f", "rawvideo", "-pixel_format", "rgb24" if message.encoding == "rgb8" else "bgr24",
                "-video_size", f"{message.width}x{message.height}",
                "-framerate", str(args.fps), "-i", "pipe:0", "-an",
                "-c:v", "libx264", "-preset", "veryfast", "-crf", "20",
                "-threads", "2", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
                str(output),
            ], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=encoder_log,
                start_new_session=True)
            worker = threading.Thread(target=encode_frames, name="video_encoder", daemon=True)
            worker.start()
        if (message.width, message.height, message.encoding) != (
                stats["width"], stats["height"], stats["encoding"]):
            errors.append("Camera dimensions or encoding changed while recording")
            stopping.set()
            return
        if last_stamp is not None:
            if stamp < last_stamp:
                errors.append("Simulation time moved backwards while recording")
                stopping.set()
                return
            stats["max_image_gap_s"] = max(stats["max_image_gap_s"], stamp - last_stamp)
        last_stamp = stamp
        stats["frames_received"] += 1
        pixels = bytes(message.data)
        if message.step != row_bytes:
            pixels = b"".join(pixels[row * message.step:row * message.step + row_bytes]
                              for row in range(message.height))
        try:
            pending.put_nowait((stamp, pixels))
        except queue.Full:
            stats["frames_queue_dropped"] += 1
        if args.duration and stamp - first_stamp >= args.duration:
            stopping.set()

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("hamr_simulation_video_recorder")
    node.create_subscription(Image, args.topic, on_image, qos_profile_sensor_data)
    exit_code = 0
    try:
        if not args.no_bridge:
            bridge = subprocess.Popen([
                "ros2", "run", "ros_gz_bridge", "parameter_bridge",
                f"{args.topic}@sensor_msgs/msg/Image[gz.msgs.Image",
                "--ros-args", "-r", "__node:=hamr_recording_image_bridge",
            ], stdout=bridge_log, stderr=subprocess.STDOUT, start_new_session=True)
        ready = False
        while not stopping.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
            if not ready and stats["frames_encoded"] > 0:
                ready = True
                if args.ready_file:
                    args.ready_file.write_text(json.dumps({
                        "pid": os.getpid(), "output": str(output),
                        "first_sim_time_s": first_stamp,
                        "width": stats["width"], "height": stats["height"],
                    }, indent=2) + "\n")
                print(f"RECORDING {output} at {args.fps} fps (simulation time)", flush=True)
            if bridge and bridge.poll() is not None:
                raise RuntimeError(f"Image bridge exited; see {bridge_log_path}")
            if time.monotonic() - started > args.wall_timeout:
                raise TimeoutError("Recording wall timeout reached")
            if first_stamp is None and time.monotonic() - started > min(90, args.wall_timeout):
                raise TimeoutError("No camera images; check Sensors plugin, topic and GZ_PARTITION")
    except Exception as exc:
        errors.append(str(exc))
        exit_code = 1
    finally:
        stopping.set()
        node.destroy_node()
        rclpy.shutdown()
        stop_process(bridge)
        if worker and worker.is_alive():
            pending.put(None, timeout=30)
            worker.join(timeout=60)
            if worker.is_alive():
                errors.append("Encoder did not drain within 60 s")
        if encoder:
            try:
                encoder.stdin.close()
            except BrokenPipeError:
                pass
            try:
                code = encoder.wait(timeout=60)
                if code:
                    errors.append(f"ffmpeg exited {code}; see {encoder_log_path}")
            except subprocess.TimeoutExpired:
                encoder.kill()
                encoder.wait()
                errors.append("ffmpeg timed out while finalizing MP4")
        encoder_log.close()
        bridge_log.close()
        if first_stamp is None or stats["frames_encoded"] == 0:
            errors.append("No video frames were encoded")
        stats.update(last_sim_time_s=last_stamp,
                     simulation_span_s=(last_stamp - first_stamp) if first_stamp is not None else 0,
                     wall_duration_s=time.monotonic() - started,
                     expected_video_duration_s=stats["frames_encoded"] / args.fps)
        if output.exists() and shutil.which("ffprobe"):
            probe = subprocess.run([
                "ffprobe", "-v", "error", "-show_entries",
                "stream=codec_name,width,height,pix_fmt,r_frame_rate,nb_frames,duration:format=duration,size",
                "-of", "json", str(output),
            ], text=True, capture_output=True, timeout=30)
            if probe.returncode:
                errors.append(f"ffprobe failed: {probe.stderr.strip()}")
            else:
                stats["ffprobe"] = json.loads(probe.stdout)
                stream = stats["ffprobe"]["streams"][0]
                if Fraction(stream["r_frame_rate"]) != args.fps:
                    errors.append("Encoded frame rate differs from requested rate")
        stats["errors"] = errors
        stats["success"] = not errors
        metadata.parent.mkdir(parents=True, exist_ok=True)
        metadata.write_text(json.dumps(stats, indent=2) + "\n")
        print(json.dumps(stats, indent=2), flush=True)
    return 1 if errors else exit_code


if __name__ == "__main__":
    raise SystemExit(main())
