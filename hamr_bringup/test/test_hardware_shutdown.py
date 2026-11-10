"""Exercise installed processes in isolated DDS domains and a PTY, never USB."""
import os
from pathlib import Path
import pty
import signal
import struct
import subprocess
import sys
import termios
import threading
import time
import zlib

import pytest

pytest.importorskip("rclpy")
from ament_index_python.packages import get_package_prefix


def environment(tmp_path):
    env = dict(os.environ, ROS_DOMAIN_ID=str(180 + os.getpid() % 30),
               ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST",
               ROS_LOG_DIR=str(tmp_path / "roslogs"), PYTHONUNBUFFERED="1")
    env.pop("ROS_LOCALHOST_ONLY", None)
    return env


def wait_ready(process, log, marker):
    end = time.monotonic() + 10
    while time.monotonic() < end:
        if marker in log.read_text():
            return
        assert process.poll() is None, log.read_text()
        time.sleep(0.02)
    pytest.fail(log.read_text())


def stop(process, signum, log):
    start = time.monotonic()
    process.send_signal(signum)
    process.wait(timeout=3)
    assert time.monotonic() - start < 2.0
    assert process.returncode == 0, log.read_text()
    output = log.read_text()
    for unwanted in ("Traceback", "context is invalid", "terminate called", "exception:"):
        assert unwanted not in output, output


@pytest.mark.parametrize("signum", [signal.SIGINT, signal.SIGTERM])
@pytest.mark.parametrize("module,marker", [
    ("hamr_control.hamr_controller", "HAMR Controller has been started"),
    ("hamr_odometry.holonomic_odom_node", "HolonomicOdomNode ready"),
])
def test_python_nodes_exit_cleanly(tmp_path, module, marker, signum):
    log = tmp_path / "process.log"
    with log.open("w") as stream:
        process = subprocess.Popen([sys.executable, "-m", module], env=environment(tmp_path),
                                   stdout=stream, stderr=subprocess.STDOUT)
        try:
            wait_ready(process, log, marker)
            stop(process, signum, log)
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()


@pytest.mark.parametrize("signum", [signal.SIGINT, signal.SIGTERM])
@pytest.mark.parametrize("mode", ["quiet", "streaming", "blocked"])
def test_relay_shutdown_with_quiet_streaming_or_blocked_serial_port(tmp_path, signum, mode):
    binary = Path(get_package_prefix("hamr_uros_bridge")) / "lib/hamr_uros_bridge/relay_node"
    master, slave = pty.openpty()
    log = tmp_path / "relay.log"
    stopping = threading.Event()
    reader_input = None
    with log.open("w") as stream:
        process = subprocess.Popen([str(binary), "--ros-args", "-p",
                                    f"serial_port:={os.ttyname(slave)}"],
                                   env=environment(tmp_path), stdout=stream,
                                   stderr=subprocess.STDOUT)
        try:
            wait_ready(process, log, "Serial open:")
            os.set_blocking(master, False)
            if mode == "blocked":
                # Deterministic EAGAIN, including the shutdown zero writes.
                termios.tcflow(slave, termios.TCOOFF)
                time.sleep(0.1)
            if mode == "streaming":
                def feed_encoders():
                    sequence = 0
                    while not stopping.wait(0.002):
                        packet = struct.pack("<HHHIQiii", 0xCAFE, 4, 3, sequence,
                                             time.monotonic_ns(), sequence, sequence, 0)
                        try:
                            os.write(master, packet + struct.pack("<H", zlib.crc32(packet) & 0xFFFF))
                        except BlockingIOError:
                            pass
                        sequence += 1
                reader_input = threading.Thread(target=feed_encoders)
                reader_input.start()
                time.sleep(0.1)
            stop(process, signum, log)
            if mode != "blocked":
                packets = bytearray()
                while True:
                    try:
                        packets.extend(os.read(master, 65536))
                    except BlockingIOError:
                        break
                size = 32
                assert len(packets) >= 10 * size
                assert len(packets) % size == 0
                for offset in range(len(packets) - 10 * size, len(packets), size):
                    packet = packets[offset:offset + size]
                    magic, version, kind, _, _, left, right, turret, crc = struct.unpack("<HHHIQfffH", packet)
                    assert (magic, version, kind) == (0xCAFE, 4, 0x11)
                    assert (left, right, turret) == (0.0, 0.0, 0.0)
                    assert crc == zlib.crc32(packet[:-2]) & 0xFFFF
        finally:
            stopping.set()
            if reader_input is not None:
                reader_input.join(timeout=1)
            if process.poll() is None:
                process.kill()
                process.wait()
            os.close(master)
            os.close(slave)
