"""Isolated Gazebo process lifetime and the repository's terrain catalogue.

This module deliberately has no ROS or Qt import so discovery and lifecycle
validation also work in an ordinary Python terminal.
"""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import json
import math
import os
from pathlib import Path
import random
import re
import signal
import subprocess
import time
import uuid


@dataclass(frozen=True)
class Terrain:
    id: str
    title: str
    world: Path
    bounds: tuple[float, float, float, float]
    spawn: tuple[float, float, float, float]
    height_grid: Path | None = None
    description: str = ""


def discover_terrains(repo_root):
    """Read canonical map metadata, retaining only worlds present on disk."""
    root = Path(repo_root).resolve()
    config = root / "maps/config"
    terrains = {}
    for path in sorted(config.glob("*.json")):
        try:
            value = json.loads(path.read_text())
            entries = value if isinstance(value, list) else [value]
            for item in entries:
                if not isinstance(item, dict) or not all(k in item for k in ("id", "world", "bounds_m")):
                    continue
                world = (config / item["world"]).resolve()
                bounds = tuple(float(v) for axis in ("x", "y") for v in item["bounds_m"][axis])
                spawn = item.get("spawn", {})
                pose = tuple(float(spawn.get(k, default)) for k, default in
                             (("x", 0.), ("y", -2.8), ("z", 0.), ("yaw", math.pi / 2)))
                if (len(bounds) != 4 or not all(math.isfinite(v) for v in (*bounds, *pose))
                        or bounds[0] >= bounds[1] or bounds[2] >= bounds[3] or not world.is_file()):
                    continue
                grid = (config / item["height_grid"]).resolve() if item.get("height_grid") else None
                terrains[item["id"]] = Terrain(str(item["id"]), str(item.get("title", item["id"])),
                    world, bounds, pose, grid if grid and grid.is_file() else None,
                    str(item.get("description", "")))
        except (OSError, ValueError, KeyError, TypeError):
            continue
    flat_world = root / "hamr_swivel_caster/worlds/swivel_caster_test.sdf"
    if flat_world.is_file():
        terrains["flat"] = Terrain("flat", "Flat test floor", flat_world,
                                   (-3., 3., -3., 3.), (0., 0., 0., 0.))
    return sorted(terrains.values(), key=lambda item: (item.id != "flat", item.title.casefold()))


def _process_group_alive(group):
    """Count executable members, including children whose launcher has exited.

    An orphaned zombie remains in its process group until the host reaps it,
    but it cannot consume CPU or flush a log. Linux /proc lets shutdown avoid
    spending every escalation grace period waiting for those dead members.
    """
    proc = Path("/proc")
    if proc.is_dir():
        inaccessible = False
        for entry in proc.iterdir():
            if not entry.name.isdigit():
                continue
            try:
                # The parenthesized command can contain spaces or ')'.
                fields = (entry / "stat").read_text().rsplit(")", 1)[1].split()
                if int(fields[2]) == group and fields[0] not in ("Z", "X"):
                    return True
            except (FileNotFoundError, ProcessLookupError):
                continue
            except (PermissionError, ValueError, IndexError):
                inaccessible = True
        if not inaccessible:
            return False
    try:
        os.killpg(group, 0)
        return True
    except ProcessLookupError:
        return False


def stop_process_group(process, grace=10.):
    """Stop an owned start_new_session group, allowing every child to flush.

    ros2 launch may exit before its Gazebo server. Reaping the launcher alone
    does not mean the group is gone: inspect and escalate remaining members
    even when Popen.poll() already reports the launcher has exited.
    """
    if process is None:
        return
    process.poll()  # Reap our direct child promptly if it has already exited.
    for sig, timeout in ((signal.SIGINT, grace), (signal.SIGTERM, 3.), (signal.SIGKILL, 2.)):
        if not _process_group_alive(process.pid):
            return
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            return
        deadline = time.monotonic() + timeout
        while True:
            process.poll()
            if not _process_group_alive(process.pid):
                return
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            time.sleep(min(.05, remaining))


class SimulationSession:
    """One owned Gazebo world; instantiate before initializing rclpy.

    Apply ``session.env``'s ROS_DOMAIN_ID and GZ_PARTITION to the UI process
    before rclpy.init. The companion and its children then share an isolated
    graph. Calls starting/stopping processes should run off the Qt UI thread.
    """

    def __init__(self, repo_root, output_root=None, env=None, domain_id=None):
        self.root = Path(repo_root).expanduser().resolve()
        self.output_root = Path(output_root or Path.home() / "Videos/hamr_sim_ui").expanduser().resolve()
        self.env = dict(os.environ if env is None else env)
        domain = random.SystemRandom().randrange(100, 220) if domain_id is None else int(domain_id)
        if not 0 <= domain <= 232:
            raise ValueError("ROS domain must be in [0, 232]")
        self.env.update(ROS_DOMAIN_ID=str(domain), ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST",
                        ROS_LOCALHOST_ONLY="1", GZ_PARTITION="hamr_ui_" + uuid.uuid4().hex)
        resources = [str(self.root), str(self.root / "maps"), self.env.get("GZ_SIM_RESOURCE_PATH", "")]
        self.env["GZ_SIM_RESOURCE_PATH"] = ":".join(filter(None, resources))
        # The rootless Gazebo bundle relocates libraries but its SQLite logger
        # still needs the installed transport schema directory explicitly.
        sql_paths = sorted((Path.home() / ".cache/hamr-gazebo-root/opt/ros/jazzy/opt").glob(
            "gz_transport_vendor/share/gz/gz-transport*/sql"))
        if not self.env.get("GZ_TRANSPORT_LOG_SQL_PATH") and sql_paths:
            self.env["GZ_TRANSPORT_LOG_SQL_PATH"] = str(sql_paths[-1])
        self.terrains = {item.id: item for item in discover_terrains(self.root)}
        self.process = None
        self._bridge = None
        self._streams = []
        self.run_dir = None
        self.world_name = None
        self.terrain = None
        self.caster = "ball"
        self.cameras = False
        self.paused = False
        self.replay = False

    @property
    def running(self):
        return self.process is not None and self.process.poll() is None

    @property
    def gazebo_log(self):
        return self.run_dir / "gazebo_log" if self.run_dir else None

    def _spawn(self, command, logfile):
        stream = Path(logfile).open("w")
        self._streams.append(stream)
        try:
            return subprocess.Popen(command, cwd=self.root, env=self.env, stdout=stream,
                                    stderr=subprocess.STDOUT, start_new_session=True)
        except Exception:
            stream.close()
            raise

    def start(self, terrain_id, gui=True, caster="ball", cameras=False):
        from .world import prepare_world, write_gui_config

        if self.running:
            raise RuntimeError("Stop the current world before loading a map")
        if terrain_id not in self.terrains:
            raise ValueError(f"Unknown terrain {terrain_id!r}")
        if caster not in ("ball", "swivel"):
            raise ValueError("Caster must be ball or swivel")
        self.stop()
        terrain = self.terrains[terrain_id]
        self.run_dir = self.output_root / datetime.now().strftime("session_%Y%m%d_%H%M%S_%f")
        self.run_dir.mkdir(parents=True, exist_ok=False)
        self.terrain, self.replay, self.paused = terrain, False, False
        self.caster, self.cameras = caster, bool(cameras)
        world = self.run_dir / "world.sdf"
        self.world_name = prepare_world(terrain.world, world, terrain.bounds,
                                        log_path=self.gazebo_log, cameras=self.cameras)
        gui_config = write_gui_config(self.run_dir / "gui.config", terrain.bounds)
        x, y, z, yaw = terrain.spawn
        command = ["ros2", "launch", str(self.root / "hamr_terrain/launch/terrain_sim.launch.py"),
                   f"caster:={caster}", f"closeup_camera:={str(self.cameras).lower()}",
                   f"gui:={str(bool(gui)).lower()}",
                   f"world:={world}", f"gui_config:={gui_config}",
                   f"spawn_x:={x}", f"spawn_y:={y}", f"spawn_z:={z}", f"initial_yaw:={yaw}"]
        manifest = {"terrain": terrain.id, "caster": caster, "cameras": self.cameras,
                    "world_name": self.world_name, "source_world": str(terrain.world),
                    "gazebo_log": str(self.gazebo_log), "scene_log_scope": "full simulation session",
                    "spawn": list(terrain.spawn), "command": command,
                    "environment": {key: self.env[key] for key in ("ROS_DOMAIN_ID", "GZ_PARTITION")}}
        (self.run_dir / "session.json").write_text(json.dumps(manifest, indent=2) + "\n")
        try:
            self.process = self._spawn(command, self.run_dir / "gazebo.log")
            if self.cameras:
                self._bridge = self._spawn(["ros2", "run", "ros_gz_bridge", "parameter_bridge",
                    f"/hamr/{caster}_caster/closeup/image@sensor_msgs/msg/Image[gz.msgs.Image",
                    "/hamr/recording_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
                    "--ros-args", "-r", "__node:=hamr_ui_cameras"], self.run_dir / "cameras.log")
        except Exception:
            self.stop()
            raise
        return self.run_dir

    def poll(self):
        """Return a concise asynchronous failure, or None while healthy/idle."""
        for label, process in (("Gazebo", self.process), ("Camera bridge", self._bridge)):
            if process is not None and process.poll() is not None:
                return f"{label} exited ({process.returncode}); see {self.run_dir}"
        return None

    def set_paused(self, paused):
        if not self.running or not self.world_name:
            raise RuntimeError("No simulation is running")
        result = subprocess.run(["gz", "service", "-s", f"/world/{self.world_name}/control",
            "--reqtype", "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean",
            "--timeout", "3000", "--req", f"pause: {str(bool(paused)).lower()}"],
            env=self.env, capture_output=True, text=True, timeout=5.)
        if result.returncode or "data: true" not in result.stdout:
            raise RuntimeError("Gazebo did not acknowledge pause/resume: " +
                               (result.stderr.strip() or result.stdout.strip()))
        self.paused = bool(paused)

    def placement_pose(self, x, y, yaw):
        """An upright root pose above the highest ground under the vehicle.

        Both COMPA caster models use a ground-level base_footprint root. The
        footprint envelope covers their wheels and passive caster assemblies;
        checking every height-grid vertex in its bounding rectangle avoids
        inserting a wheel through a narrow terrain peak between sample points.
        Gravity settles the small clearance after the simulation resumes.
        """
        x, y, yaw = float(x), float(y), float(yaw)
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            raise ValueError("Car position and heading must be finite")
        if self.terrain is None:
            raise RuntimeError("Load a terrain before placing the car")
        xmin, xmax, ymin, ymax = self.terrain.bounds
        if not (xmin <= x <= xmax and ymin <= y <= ymax):
            raise ValueError("Place the car inside the terrain map")
        height = 0.
        if self.terrain.height_grid:
            import numpy as np
            with np.load(self.terrain.height_grid) as grid:
                gx, gy, gz = (grid[key] for key in ("x", "y", "z"))
                if (gx.ndim != 1 or gy.ndim != 1 or len(gx) < 2 or len(gy) < 2
                        or gz.shape != (len(gy), len(gx))
                        or not all(np.isfinite(a).all() for a in (gx, gy, gz))
                        or not (np.diff(gx) > 0).all() or not (np.diff(gy) > 0).all()):
                    raise ValueError("Terrain height grid is invalid")
                # Conservative axis-aligned bounds around the rotated chassis.
                c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
                rx, ry = .55*c + .45*s, .55*s + .45*c
                i0 = max(0, int(np.searchsorted(gx, x-rx))-1)
                i1 = min(len(gx), int(np.searchsorted(gx, x+rx))+1)
                j0 = max(0, int(np.searchsorted(gy, y-ry))-1)
                j1 = min(len(gy), int(np.searchsorted(gy, y+ry))+1)
                height = float(np.max(gz[j0:j1, i0:i1]))
        return {"x": x, "y": y, "z": height + .04,
                "yaw": math.atan2(math.sin(yaw), math.cos(yaw))}

    def _read_paused(self):
        """Read Gazebo's state, including pauses from its own viewport controls."""
        result = subprocess.run(["gz", "topic", "-t", f"/world/{self.world_name}/stats",
            "--echo", "--num", "1", "--json-output"], env=self.env,
            capture_output=True, text=True, timeout=5.)
        if result.returncode:
            raise RuntimeError("Cannot read Gazebo pause state: " + result.stderr.strip())
        try:
            state = json.loads(result.stdout)
            if not isinstance(state, dict) or not any(key in state for key in ("simTime", "sim_time")):
                raise ValueError("missing simulation time")
            paused = state.get("paused", False)
            if not isinstance(paused, bool):
                raise ValueError("invalid pause flag")
        except (ValueError, TypeError) as exc:
            raise RuntimeError("Cannot read Gazebo pause state") from exc
        self.paused = paused
        return paused

    def _model_pose(self):
        result = subprocess.run(["gz", "model", "--model", "compa", "--pose"],
            env=self.env, capture_output=True, text=True, timeout=8.)
        match = re.search(r"Pose[^\n]*\n\s*\[([^\]]+)\]\s*\[([^\]]+)\]", result.stdout)
        if result.returncode or not match:
            raise RuntimeError("Gazebo did not report the car's placed pose")
        values = tuple(float(value) for part in match.groups() for value in part.split())
        if len(values) != 6 or not all(math.isfinite(value) for value in values):
            raise RuntimeError("Gazebo reported an invalid car pose")
        return values

    def _wait_stopped(self, feedback, timeout):
        deadline, stopped_since, settled_pose = time.monotonic() + timeout, None, None
        while time.monotonic() < deadline:
            if not self.running:
                raise RuntimeError("Simulation stopped before the car could be placed")
            state = feedback()
            velocity, stamp = state.get("velocity"), state.get("odom_time")
            attitude = tuple(state.get(key) for key in ("z", "roll", "pitch"))
            age = state.get("feedback_age_s")
            ready = (state.get("feedback_ready") and velocity is not None and len(velocity) == 3
                     and stamp is not None and age is not None and 0 <= age <= 3.
                     and all(v is not None and math.isfinite(v)
                             for v in (*velocity, stamp, *attitude)))
            if ready and math.hypot(*velocity[:2]) < .025 and abs(velocity[2]) < .05:
                if stopped_since is None:
                    stopped_since, settled_pose = stamp, attitude
                elif (abs(attitude[0]-settled_pose[0]) > .002
                      or max(abs(a-b) for a, b in zip(attitude[1:], settled_pose[1:])) > .005):
                    # Do not carry falling or pitching momentum across the map.
                    stopped_since, settled_pose = stamp, attitude
                elif stamp - stopped_since >= .2:
                    return state
            else:
                stopped_since = None
            time.sleep(.04)
        raise RuntimeError("Car did not stop; placement cancelled. Stop driving and try again.")

    def reposition(self, x, y, yaw=None, *, feedback, timeout=20.):
        """Move the whole articulated car, preserving joints and pause state.

        The caller must halt its controller first and prevent new commands
        until this call returns. ``feedback`` is ``controller.snapshot``.
        Gazebo's pose service preserves momentum, so require fresh stationary
        feedback before teleporting instead of pretending to reset velocity.
        An initially paused world briefly runs to apply the zero commands and
        verify settling, then returns to paused at the requested pose.
        """
        if not self.running or not self.world_name or self.replay:
            raise RuntimeError("Car placement requires a running live simulation")
        if not callable(feedback) or not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("Placement requires live feedback and a positive timeout")
        before = feedback()
        if yaw is None:
            yaw = before.get("yaw")
        if yaw is None:
            raise RuntimeError("Wait for live car heading before placing the car")
        target = self.placement_pose(x, y, yaw)
        was_paused = self._read_paused()
        event = {"event": "car_reposition", "target": target, "caster": self.caster,
                 "before_position": before.get("position"), "sim_time": before.get("sim_time"),
                 "was_paused": was_paused, "status": "failed"}
        try:
            if was_paused:
                self.set_paused(False)
            stopped = self._wait_stopped(feedback, timeout)
            self.set_paused(True)
            event["settled_velocity"] = stopped["velocity"]
            request = (f'name: "compa" position {{x: {target["x"]:.12g} y: {target["y"]:.12g} '
                       f'z: {target["z"]:.12g}}} orientation {{z: {math.sin(target["yaw"]/2):.12g} '
                       f'w: {math.cos(target["yaw"]/2):.12g}}}')
            result = subprocess.run(["gz", "service", "-s", f"/world/{self.world_name}/set_pose/blocking",
                "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean", "--timeout", "5000",
                "--req", request], env=self.env, capture_output=True, text=True, timeout=7.)
            if result.returncode or "data: true" not in result.stdout:
                raise RuntimeError("Gazebo did not acknowledge car placement: " +
                                   (result.stderr.strip() or result.stdout.strip()))
            actual = self._model_pose()
            error = math.dist(actual[:3], (target["x"], target["y"], target["z"]))
            yaw_error = math.atan2(math.sin(actual[5]-target["yaw"]), math.cos(actual[5]-target["yaw"]))
            if error > .002 or abs(yaw_error) > .002:
                raise RuntimeError("Gazebo placement could not be verified; keep the car stopped")
            event.update(status="placed", actual_pose=list(actual))
            return target
        except Exception as exc:
            event["error"] = str(exc)
            raise
        finally:
            try:
                if self.running and self.paused != was_paused:
                    self.set_paused(was_paused)
            finally:
                if self.run_dir:
                    with (self.run_dir / "events.jsonl").open("a") as stream:
                        stream.write(json.dumps(event, allow_nan=False) + "\n")

    def start_replay(self, path, gui=True):
        """Replay finalized native state history; ROS bag is intentionally separate.

        Native state covers the full session. A recording toggle covers a shorter
        ROS/video interval, so playing its bag simultaneously would give false
        synchronization. Playback therefore has no live ROS telemetry/controller.
        """
        from .world import write_gui_config

        path = Path(path).expanduser().resolve()
        if path.is_file():
            path = path.parent
        session_dir = path
        log = path / "gazebo_log"
        if (path / "state.tlog").is_file():
            log, session_dir = path, path.parent
        elif not log.is_dir():
            for name in ("recording.json", "session.json"):
                metadata = path / name
                if metadata.is_file():
                    info = json.loads(metadata.read_text())
                    context = info.get("session", {})
                    raw = (info.get("gazebo_log") or info.get("gazebo_log_path")
                           or context.get("gazebo_log") or context.get("gazebo_log_path"))
                    if raw:
                        candidate = Path(raw).expanduser()
                        log = candidate if candidate.is_absolute() else path / candidate
                        session_dir = log.parent
                        break
        if not log.is_dir():
            raise ValueError(f"No native Gazebo scene log found at {path}")
        self.stop()  # Flush an active journal before Gazebo tries to read it.
        if not (log / "state.tlog").is_file() or (log / "state.tlog").stat().st_size == 0:
            raise ValueError(f"Gazebo scene log is not finalized: {log}")
        manifest = session_dir / "session.json"
        info = json.loads(manifest.read_text()) if manifest.is_file() else {}
        self.run_dir, self.replay, self.paused = session_dir, True, False
        self.caster, self.cameras = info.get("caster", "ball"), bool(info.get("cameras", True))
        # Gazebo's LogPlayback system creates a playback world called default;
        # the source world's historical name is retained only in log records.
        self.world_name = "default"
        self.terrain = self.terrains.get(info.get("terrain"))
        command = ["gz", "sim", "-r", "-v", "3", "--playback", str(log)]
        if not gui:
            command += ["-s", "--headless-rendering"]
        else:
            config = write_gui_config(session_dir / "playback_gui.config",
                                      self.terrain.bounds if self.terrain else None,
                                      playback=True)
            command += ["--gui-config", str(config)]
        self.process = self._spawn(command, session_dir / "playback.log")
        return session_dir

    def stop(self):
        stop_process_group(self._bridge, grace=4.)
        stop_process_group(self.process, grace=15.)
        self._bridge = self.process = None
        for stream in self._streams:
            stream.close()
        self._streams.clear()
        self.paused = False

    close = stop
