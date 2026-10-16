#!/usr/bin/env python3
"""Preview the four-active-ball concept in native Gazebo Harmonic.

This is an ideal, pose-driven kinematic visualization. The motion comes from
active_ball_trajectory.py; no contact, actuator, traction, or controller model
is inferred from it. Existing passive-caster simulations are independent.

Source scripts/env.sh and use /usr/bin/python3 on the development workstation.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

from active_ball_trajectory import (BALL_OFFSETS, BALL_RADIUS, BOX_HALF_EXTENT,
                                    TARGETS, DemoTrajectory)

WORLD_NAME = "hamr_active_ball_demo"
ROBOT_NAME = "active_ball_chassis"
OFFSETS = BALL_OFFSETS
RADIUS = BALL_RADIUS


def add(parent, tag, text=None, **attributes):
    element = ET.SubElement(parent, tag, attributes)
    if text is not None:
        element.text = str(text)
    return element


def numbers(values):
    return " ".join(f"{value:.10g}" for value in values)


def visual(link, name, pose, shape, dimensions, color):
    item = add(link, "visual", name=name)
    add(item, "pose", numbers(pose))
    geometry = add(item, "geometry")
    primitive = add(geometry, shape)
    if shape == "box":
        add(primitive, "size", numbers(dimensions))
    elif shape == "cylinder":
        add(primitive, "radius", dimensions[0])
        add(primitive, "length", dimensions[1])
    elif shape == "sphere":
        add(primitive, "radius", dimensions[0])
    elif shape == "mesh":
        add(primitive, "uri", dimensions[0])
    else:
        raise ValueError(f"Unsupported primitive {shape}")
    material = add(item, "material")
    add(material, "ambient", numbers(color))
    add(material, "diffuse", numbers(color))
    add(material, "specular", ".12 .12 .12 1")
    return item


def static_model(world, name, pose=(0, 0, 0, 0, 0, 0)):
    model = add(world, "model", name=name)
    add(model, "static", "true")
    add(model, "pose", numbers(pose))
    return add(model, "link", name="body")


def write_ball_wedge(path):
    """Write a smooth sphere wedge used by all eight colored ball stripes."""
    vertices = []
    latitude_count, longitude_count = 32, 6
    for lat in range(latitude_count + 1):
        theta = math.pi * lat / latitude_count
        for lon in range(longitude_count + 1):
            phi = math.pi / 4 * lon / longitude_count
            vertices.append((math.sin(theta) * math.cos(phi),
                             math.sin(theta) * math.sin(phi), math.cos(theta)))
    lines = ["# Active ball visual sphere wedge; dimensions in metres."]
    lines.extend("v " + numbers(tuple(RADIUS * x for x in vertex)) for vertex in vertices)
    lines.extend("vn " + numbers(vertex) for vertex in vertices)
    for lat in range(latitude_count):
        for lon in range(longitude_count):
            a = lat * (longitude_count + 1) + lon + 1
            b = a + longitude_count + 1
            for triangle in ((a, b, a + 1), (a + 1, b, b + 1)):
                lines.append("f " + " ".join(f"{index}//{index}" for index in triangle))
    path.write_text("\n".join(lines) + "\n")


def write_world(path):
    """Generate a standalone SDF and its one local visual mesh."""
    path = Path(path).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    wedge_path = path.parent / "active_ball_stripe.obj"
    write_ball_wedge(wedge_path)
    sdf = ET.Element("sdf", version="1.9")
    world = add(sdf, "world", name=WORLD_NAME)
    physics = add(world, "physics", name="kinematic_preview", type="ignored")
    add(physics, "max_step_size", ".002")
    add(physics, "real_time_factor", "1")
    add(world, "gravity", "0 0 0")
    for filename, name in (("gz-sim-physics-system", "Physics"),
                           ("gz-sim-user-commands-system", "UserCommands"),
                           ("gz-sim-scene-broadcaster-system", "SceneBroadcaster")):
        add(world, "plugin", filename=filename, name=f"gz::sim::systems::{name}")
    scene = add(world, "scene")
    add(scene, "ambient", ".6 .6 .6 1")
    add(scene, "background", ".06 .085 .12 1")
    add(scene, "shadows", "false")
    light = add(world, "light", name="studio", type="directional")
    add(light, "pose", "0 0 8 0 0 0")
    add(light, "diffuse", ".9 .9 .9 1")
    add(light, "specular", ".2 .2 .2 1")
    add(light, "direction", "-.3 .4 -1")
    add(light, "cast_shadows", "false")
    floor = static_model(world, "demo_floor")
    visual(floor, "floor", (0, 0, -.012, 0, 0, 0), "box", (4.5, 4.5, .02), (.13, .17, .22, 1))
    for i in range(-4, 5):
        visual(floor, f"grid_x_{i}", (i * .4, 0, .0005, 0, 0, 0), "box",
               (.006, 3.8, .001), (.22, .28, .34, 1))
        visual(floor, f"grid_y_{i}", (0, i * .4, .0005, 0, 0, 0), "box",
               (3.8, .006, .001), (.22, .28, .34, 1))
    for axis in range(2):
        for sign in (-1, 1):
            pose = [0, 0, .003, 0, 0, 0]
            pose[axis] = sign * BOX_HALF_EXTENT
            side = 2 * BOX_HALF_EXTENT
            size = (.014, side, .002) if axis == 0 else (side, .014, .002)
            visual(floor, f"boundary_{axis}_{sign}", pose, "box", size, (.1, .7, .78, 1))
    destinations = [target["xy"] for target in TARGETS]
    for i, (x, y) in enumerate(destinations):
        visual(floor, f"spoke_{i}", (x / 2, y / 2, .002, 0, 0, math.atan2(y, x)),
               "box", (math.hypot(x, y), .01, .001), (.33, .41, .5, 1))
        color = (.65, .48, .96, 1) if x and y else (.12, .78, .8, 1)
        visual(floor, f"destination_{i}", (x, y, .006, 0, 0, 0), "cylinder", (.075, .006), color)
    visual(floor, "center", (0, 0, .006, 0, 0, 0), "cylinder", (.08, .006), (1, .68, .2, 1))

    chassis = static_model(world, ROBOT_NAME)
    visual(chassis, "chassis", (0, 0, .28, 0, 0, 0), "box", (.58, .46, .12), (.12, .2, .29, 1))
    visual(chassis, "roof", (0, 0, .3625, 0, 0, 0), "box", (.42, .34, .055), (.75, .83, .89, 1))
    # The two diagonal strips form a permanent arrow pointing to body +X.
    visual(chassis, "front_arrow_stem", (.01, 0, .393, 0, 0, 0), "box", (.23, .024, .004), (.08, .5, .65, 1))
    for sign in (-1, 1):
        visual(chassis, f"front_arrow_{sign}", (.102, sign * .032, .393, 0, 0, -sign * math.pi / 4),
               "box", (.10, .024, .004), (.08, .5, .65, 1))
    for i, (x, y) in enumerate(OFFSETS):
        visual(chassis, f"mount_{i}", (x, y, .24, 0, 0, 0), "cylinder", (.072, .065), (.36, .43, .49, 1))
        visual(chassis, f"motor_x_{i}", (x, y + math.copysign(.053, y), .195, 0, math.pi / 2, 0),
               "cylinder", (.025, .115), (.98, .57, .15, 1))
        visual(chassis, f"motor_y_{i}", (x + math.copysign(.053, x), y, .195, math.pi / 2, 0, 0),
               "cylinder", (.025, .115), (.58, .66, .72, 1))
        ball = static_model(world, f"active_ball_{i}", (x, y, RADIUS, 0, 0, 0))
        for stripe in range(8):
            color = (.05, .67, .76, 1) if stripe % 2 else (.78, .86, .9, 1)
            visual(ball, f"stripe_{stripe}", (0, 0, 0, 0, 0, stripe * math.pi / 4),
                   "mesh", (wedge_path.as_uri(),), color)
    gui = add(world, "gui", fullscreen="false")
    camera = add(gui, "plugin", filename="MinimalScene", name="3D View")
    camera_ui = add(camera, "gz-gui")
    add(camera_ui, "property", "false", type="bool", key="showTitleBar")
    add(camera_ui, "property", "docked", type="string", key="state")
    add(camera, "engine", "ogre2")
    add(camera, "scene", "scene")
    add(camera, "ambient_light", ".6 .6 .6")
    add(camera, "background_color", ".06 .085 .12")
    add(camera, "camera_pose", "2.8 -3.2 3.8 0 .717 2.29")
    # No physics play/pause button: this deliberately scripted preview is
    # driven by wall time, and Gazebo physics pause would not pause it.
    for filename in ("GzSceneManager", "InteractiveViewControl"):
        plugin = add(gui, "plugin", filename=filename, name=filename)
        ui = add(plugin, "gz-gui")
        for key, kind, value in (("resizable", "bool", "false"),
                                 ("showTitleBar", "bool", "false"),
                                 ("width", "double", "5"), ("height", "double", "5"),
                                 ("state", "string", "floating")):
            add(ui, "property", value, type=kind, key=key)
    ET.indent(sdf, space="  ")
    ET.ElementTree(sdf).write(path, encoding="utf-8", xml_declaration=True)
    return path


def load_transport():
    """Support both apt packages and this workstation's extracted Gazebo."""
    prefixes = (Path.home() / ".cache/hamr-gazebo-root", Path("/"))
    for prefix in prefixes:
        for relative in ("usr/lib/python3/dist-packages",
                         "opt/ros/jazzy/opt/gz_transport_vendor/lib/python",
                         "opt/ros/jazzy/opt/gz_msgs_vendor/lib/python"):
            candidate = prefix / relative
            if candidate.is_dir() and str(candidate) not in sys.path:
                sys.path.append(str(candidate))
    try:
        from gz.transport13 import Node
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.msgs10.boolean_pb2 import Boolean
    except ImportError as exc:
        raise RuntimeError("Gazebo Harmonic Python bindings are unavailable. Source scripts/env.sh "
                           "and run this script with /usr/bin/python3.") from exc
    return Node, Pose_V, Boolean


def rotation_quaternion(matrix):
    """Convert a proper 3x3 matrix to quaternion (x, y, z, w)."""
    m = matrix
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0:
        s = math.sqrt(trace + 1) * 2
        q = ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s,
             (m[1][0] - m[0][1]) / s, s / 4)
    else:
        i = max(range(3), key=lambda j: m[j][j])
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(1 + m[i][i] - m[j][j] - m[k][k]) * 2
        parts = [0.0] * 4
        parts[i] = s / 4
        parts[j] = (m[j][i] + m[i][j]) / s
        parts[k] = (m[k][i] + m[i][k]) / s
        parts[3] = (m[k][j] - m[j][k]) / s
        q = tuple(parts)
    return q


def pose_message(sample, Pose_V):
    message = Pose_V()
    yaw = sample["yaw"]
    positions = [(ROBOT_NAME, sample["x"], sample["y"], 0,
                  (0, 0, math.sin(yaw / 2), math.cos(yaw / 2)))]
    for i, (x, y) in enumerate(OFFSETS):
        positions.append((f"active_ball_{i}",
                          sample["x"] + x * math.cos(yaw) - y * math.sin(yaw),
                          sample["y"] + x * math.sin(yaw) + y * math.cos(yaw), RADIUS,
                          rotation_quaternion(sample["ball_rotations"][i])))
    for name, x, y, z, quaternion in positions:
        pose = message.pose.add()
        pose.name = name
        pose.position.x, pose.position.y, pose.position.z = x, y, z
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion
    return message


def stop_process(process):
    if process is None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=5)
    except ProcessLookupError:
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--write-world", type=Path, help="Generate the SDF and mesh, then exit")
    parser.add_argument("--output-dir", type=Path, help="Store generated world, Gazebo log, and run report here")
    parser.add_argument("--headless", action="store_true", help="Run the native Gazebo server without its GUI")
    parser.add_argument("--speed", type=float, default=1, help="Preview playback multiplier; default: 1")
    parser.add_argument("--duration", type=float, help="Stop after this many trajectory seconds (smoke tests)")
    parser.add_argument("--fps", type=float, default=30, help="Requested pose update rate in wall time; default: 30")
    args = parser.parse_args()
    for name in ("speed", "fps", "duration"):
        value = getattr(args, name)
        if value is not None and (not math.isfinite(value) or value <= 0):
            parser.error(f"--{name} must be finite and positive")
    if args.write_world:
        print(write_world(args.write_world))
        return 0
    trajectory = DemoTrajectory()
    end_time = min(trajectory.duration, args.duration or trajectory.duration)
    Node, Pose_V, Boolean = load_transport()
    executable = shutil.which("gz")
    if executable is None:
        parser.error("gz is unavailable; source hamr_ball_caster/scripts/env.sh first")
    output = (args.output_dir or Path.home() / "Videos/hamr_sim" /
              datetime.now().strftime("active_ball_gazebo_%Y%m%d_%H%M%S")).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    world = write_world(output / "active_ball_demo.sdf")
    # The partition is private to this subprocess and transport client.
    os.environ["GZ_PARTITION"] = f"hamr_active_ball_preview_{os.getpid()}"
    command = [executable, "sim", "-r", "-v", "2"]
    if args.headless:
        command.append("-s")
    command.append(str(world))
    report = {"scope": "ideal pose-driven kinematic visualization; no contact or actuator dynamics",
              "clock": "wall-clock preview; Gazebo physics pause does not pause the scripted route",
              "world": str(world), "command": command, "trajectory_duration_s": trajectory.duration,
              "requested_end_time_s": end_time, "playback_speed": args.speed, "pose_updates": 0,
              "passed": False}
    process = None
    observed = {}
    def on_pose(message):
        for pose in message.pose:
            if pose.name == ROBOT_NAME or pose.name.startswith("active_ball_"):
                observed[pose.name] = (pose.position.x, pose.position.y, pose.position.z)
    try:
        with (output / "gazebo.log").open("w") as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            node = Node()
            service = f"/world/{WORLD_NAME}/set_pose_vector"
            initial = pose_message(trajectory.sample(0), Pose_V)
            startup = time.monotonic()
            while True:
                if process.poll() is not None:
                    raise RuntimeError(f"Gazebo exited during startup; see {output / 'gazebo.log'}")
                success, reply = node.request(service, initial, Pose_V, Boolean, 1000)
                if success and reply.data:
                    break
                if time.monotonic() - startup > 40:
                    raise RuntimeError("Gazebo set_pose_vector service did not become ready")
            print(f"Native Gazebo active-ball preview: {end_time:.2f} trajectory seconds", flush=True)
            began = time.monotonic()
            next_update = began
            last_phase = None
            while True:
                if process.poll() is not None:
                    raise RuntimeError("Gazebo closed before the trajectory finished")
                t = min((time.monotonic() - began) * args.speed, end_time)
                sample = trajectory.sample(t)
                poses = pose_message(sample, Pose_V)
                for attempt in range(3):
                    success, reply = node.request(service, poses, Pose_V, Boolean, 1000)
                    if success and reply.data:
                        break
                    time.sleep(.02)
                else:
                    raise RuntimeError(f"Gazebo pose update failed at t={t:.3f}s "
                                       f"(transport={success}, accepted={reply.data})")
                report["pose_updates"] += 1
                phase = (sample["phase"], sample["target_index"])
                if phase != last_phase:
                    print(f"  {t:6.2f}s  {sample['phase']}: {sample['target_name']}", flush=True)
                    last_phase = phase
                if t >= end_time:
                    break
                next_update += 1 / args.fps
                time.sleep(max(0, next_update - time.monotonic()))
            # Subscribe after requests stop: the Python transport callback and
            # synchronous request otherwise contend for the interpreter lock.
            node.subscribe(Pose_V, f"/world/{WORLD_NAME}/pose/info", on_pose)
            time.sleep(.5)
            errors = []
            for pose in poses.pose:
                actual = observed.get(pose.name)
                if actual is None:
                    raise RuntimeError(f"No native scene pose received for {pose.name}")
                expected = (pose.position.x, pose.position.y, pose.position.z)
                errors.append(math.dist(actual, expected))
            report.update(passed=max(errors) < 1e-5, final_trajectory_time_s=t,
                          max_final_scene_position_error_m=max(errors),
                          final_scene_positions_m=observed,
                          wall_duration_s=time.monotonic() - began)
            if not report["passed"]:
                raise RuntimeError("Native scene did not reach the final requested position")
    except (KeyboardInterrupt, RuntimeError) as exc:
        report.update(passed=False, failure=str(exc) or "Interrupted")
        print(report["failure"], file=sys.stderr)
    finally:
        stop_process(process)
        (output / "native_run_summary.json").write_text(json.dumps(report, indent=2) + "\n")
    print(f"{'PASS' if report['passed'] else 'FAIL'}: {output / 'native_run_summary.json'}")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
