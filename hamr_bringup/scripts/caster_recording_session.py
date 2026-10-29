"""Manual start/stop of available ESP recorders alongside an existing HAMR bag."""

import argparse
from datetime import datetime, timezone
import http.client
import json
import os
from pathlib import Path
import signal
import sys
import time
import uuid

import caster_recording_client as client
from caster_recording_bag import merge_session


MARKER = ".latest_caster_session.json"
STATE = "session.json"


def recorder_process(pid):
    """Identify only a ros2 bag record process owned by this user on this boot."""
    directory = Path("/proc") / str(pid)
    try:
        if directory.stat().st_uid != os.getuid():
            return None
        argv = directory.joinpath("cmdline").read_bytes().decode().rstrip("\0").split("\0")
        index = next(i for i in range(min(2, len(argv))) if Path(argv[i]).name == "ros2")
        if argv[index + 1:index + 3] != ["bag", "record"]:
            return None
        output = None
        for i, arg in enumerate(argv[index + 3:], index + 3):
            if arg in ("--output", "-o"):
                output = argv[i + 1]
            elif arg.startswith("--output="):
                output = arg.split("=", 1)[1]
        if output is None:
            return None
        bag = Path(output).expanduser()
        if not bag.is_absolute():
            bag = directory.joinpath("cwd").resolve() / bag
        fields = directory.joinpath("stat").read_text().rsplit(")", 1)[1].split()
        if fields[0] == "Z":
            return None
        return {"pid": int(pid), "start_ticks": int(fields[19]), "bag_path": str(bag.resolve()),
                "kernel_boot_id": Path("/proc/sys/kernel/random/boot_id").read_text().strip()}
    except (OSError, ValueError, StopIteration, IndexError, UnicodeError):
        return None


def active_recorder(bag):
    matches = [item for path in Path("/proc").iterdir() if path.name.isdigit()
               if (item := recorder_process(int(path.name))) and item["bag_path"] == str(bag)]
    if len(matches) != 1:
        raise client.RecordingError(f"Expected one active recorder for {bag}, found {len(matches)}. "
                                    "Launch hamr_HW with record_bag:=true first; --bag can select an exact active bag.")
    return matches[0]


def finalize_robot_bag(saved, timeout=30):
    """Gracefully stop the saved recorder only; never signal a launch/process group."""
    current = recorder_process(saved["pid"])
    if current == saved:
        # pidfd pins the process while we recheck its identity, closing the PID
        # reuse race. Raspberry Pi OS/Ubuntu kernels support pidfd on Linux.
        try:
            descriptor = os.pidfd_open(saved["pid"])
        except ProcessLookupError:
            descriptor = None
        if descriptor is not None:
            try:
                if recorder_process(saved["pid"]) == saved:
                    signal.pidfd_send_signal(descriptor, signal.SIGINT)
            except ProcessLookupError:
                pass
            finally:
                os.close(descriptor)
        deadline = time.monotonic() + timeout
        while recorder_process(saved["pid"]) == saved:
            if time.monotonic() >= deadline:
                raise client.RecordingError("Robot recorder has not finished closing; rerun stop after it exits")
            time.sleep(.1)
    # A PID may now belong to another process: never signal that replacement.
    bag = Path(saved["bag_path"])
    if any((item := recorder_process(int(path.name))) and item["bag_path"] == str(bag)
           for path in Path("/proc").iterdir() if path.name.isdigit()):
        raise client.RecordingError("A different recorder is using this bag; refusing to merge a changing source")
    if not (bag / "metadata.yaml").is_file():
        raise client.RecordingError(f"Robot bag is not finalized at {bag}; stop its recorder cleanly before retrying")


def select_devices(args, configuration, transport):
    raw = client.explicit_nodes(args.node, transport) if args.node else client.discover(
        args.discovery_address or configuration.get("discovery_address", "255.255.255.255"),
        configuration.get("discovery_port", 5010), args.discovery_timeout)
    return client.validate_inventory(raw, inventory=configuration.get("devices", []), require_all=False)


def load_state(session):
    state = json.loads((session.directory / STATE).read_text())
    if (state.get("version") != 1 or state.get("run_id") != session.manifest["run_id"] or
            state["robot_recorder"]["bag_path"] != session.manifest["bag_path"]):
        raise client.RecordingError("Saved unified session identity does not match its encoder manifest")
    return state


def start(args, root, configuration, transport):
    marker = root / MARKER
    if marker.exists():
        previous_run = json.loads(marker.read_text())["run_id"]
        previous = client.Session.load(root, previous_run, caster_type="ball")
        previous_state = load_state(previous)
        failed_before_commands = previous_state.get("failed_before_commands", False)
        # A stopped session may still have unread ESP logs after a failed
        # download. New starts explicitly discard those logs; keep only the
        # guard against replacing a session whose robot bag is still active.
        previous_bag_finalized = previous_state.get("robot_bag_finalized_epoch_ns")
        if (previous_state["state"] not in ("complete", "complete_with_errors") and
                not failed_before_commands and not previous_bag_finalized):
            raise client.RecordingError(f"Session {previous_run} still needs stopping/recovery; "
                                        f"run stop_encoder_recording {previous_run} first")
        if previous_state["state"] == "needs_recovery" and previous_bag_finalized:
            print(f"Previous session {previous_run} is incomplete; its remaining logs on selected ESPs "
                  f"will be discarded. Pi files retained at {previous.directory}", flush=True)
    bag = client.resolve_bag(root, args.bag)
    process = active_recorder(bag)
    devices = select_devices(args, configuration, transport)
    if not devices:
        raise client.RecordingError("No available ball-caster recorders responded; nothing started. Check ESP power/Wi-Fi or use --node.")
    selected = {(item["wheel_id"], item["node_id"]) for item in devices}
    missing = [f"{wheel}/{role}" for wheel in client.WHEELS for node, role in client.ROLES.items()
               if (wheel, node) not in selected]
    run = args.run_id or datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ_") + uuid.uuid4().hex[:8]
    session = client.Session.create(root, run, devices, bag=bag, transport=transport,
                                    discard_existing=True)
    state = {"version": 1, "run_id": run, "robot_recorder": process, "state": "starting",
             "missing_at_start": missing, "created_epoch_ns": time.time_ns(), "errors": []}
    client.write_json(session.directory / STATE, state)
    # Publish recovery identity BEFORE sending any prepare/start commands.
    client.write_json(root / MARKER, {"version": 1, "run_id": run})
    print(f"Session: {run}\nRobot bag: {bag}\nFiles: {session.directory}", flush=True)
    print("Selected: " + ", ".join(f"{item['wheel_id']}/{client.ROLES[item['node_id']]}" for item in devices), flush=True)
    if missing:
        print("Unavailable (not recorded): " + ", ".join(missing), flush=True)
    print("Clearing previous recordings on selected ESPs, including undownloaded logs, "
          "before starting this run. Existing Pi files are retained.", flush=True)
    with client.run_lock(session.directory):
        if recorder_process(process["pid"]) != process:
            raise client.RecordingError("Robot recorder exited during discovery; no ESP start commands were sent")
        try:
            session.start()
        except Exception as error:
            state.update(state="start_failed", errors=[str(error)],
                         failed_before_commands=str(error).startswith("recording preflight failed before prepare/start:"))
            client.write_json(session.directory / STATE, state)
            raise
        state.update(state="recording", started_epoch_ns=time.time_ns())
        client.write_json(session.directory / STATE, state)
    print(f"Recording on {len(devices)} ESPs. No background client is required.\n"
          f"When done: ros2 run hamr_bringup stop_encoder_recording {run}", flush=True)
    return 0


def stop(args, root, configuration, transport):
    run = args.run_id
    if run is None:
        marker = json.loads((root / MARKER).read_text())
        if marker.get("version") != 1:
            raise client.RecordingError("Invalid latest unified-session marker; supply a run ID")
        run = marker["run_id"]
    run = client.run_id(run)
    directory = root / ("encoder_" + run)
    with client.run_lock(directory):
        session = client.Session.load(root, run, caster_type="ball", transport=transport)
        state = load_state(session)
        if state["state"] in ("complete", "complete_with_errors"):
            print(f"Saved combined rosbag: {merge_session(session, state)}")
            if state["state"] == "complete_with_errors":
                print("Recording incomplete: " + "; ".join(state["errors"]), file=sys.stderr)
            return 0 if state["state"] == "complete" else 2
        state.update(state="stopping", stop_requested_epoch_ns=time.time_ns())
        client.write_json(directory / STATE, state)
        errors = []
        try:
            if not all(item.get("acknowledged") for item in session.manifest["devices"]):
                try:
                    session.refresh_endpoints(select_devices(args, configuration, transport))
                except (OSError, ValueError, client.RecordingError, http.client.HTTPException) as error:
                    print(f"Rediscovery failed; trying saved endpoints: {error}", file=sys.stderr)
            print("Stopping ESPs, collecting final clock probes, and downloading verified logs...", flush=True)
            session.stop()
        except (OSError, ValueError, client.RecordingError, http.client.HTTPException) as error:
            errors.append(str(error))
        # Finalize the exact associated bag even if a board needs recovery.
        # The hardware launch, controller, and trajectory nodes keep running.
        bag_finalized = False
        try:
            print("Finalizing the associated robot bag (hardware nodes keep running)...", flush=True)
            finalize_robot_bag(state["robot_recorder"])
            state["robot_bag_finalized_epoch_ns"] = time.time_ns()
            bag_finalized = True
        except (OSError, client.RecordingError) as error:
            errors.append(str(error))
        state.update(state="downloaded", errors=errors)
        client.write_json(directory / STATE, state)
        try:
            if not bag_finalized:
                raise client.RecordingError("Robot bag must finish closing before merge")
            print("Creating combined rosbag with acquisition-time encoder messages...", flush=True)
            combined = merge_session(session, state)
            state.update(state="complete_with_errors" if errors else "complete",
                         combined_bag=str(combined), finished_epoch_ns=time.time_ns())
            print(f"Saved combined rosbag: {combined}\nOriginal logs and timing report: {directory}", flush=True)
        except (OSError, ValueError, RuntimeError, KeyError) as error:
            errors.append(str(error))
            state.update(state="needs_recovery", errors=errors)
        client.write_json(directory / STATE, state)
        if errors:
            if state["state"] == "complete_with_errors":
                raise client.RecordingError("Combined bag saved, but recording incomplete: " + "; ".join(errors))
            raise client.RecordingError("; ".join(errors) + f". Files preserved at {directory}; rerun this stop command to recover.")
    return 0


def main(command, argv=None):
    parser = argparse.ArgumentParser(description=(
        "Erase previous onboard logs (including undownloaded data) and start available front-left/right "
        "ESP recorders after hamr_HW (record_bag:=true). Existing Pi files are retained."
        if command == "start" else
        "Stop/download the saved ESP session, finalize its robot bag, and merge. Hardware nodes keep running."))
    parser.add_argument("run_id", nargs="?", help="start: unique ID; stop: defaults to latest unified session")
    parser.add_argument("--root", type=Path, default=client.default_root())
    parser.add_argument("--config", type=Path, default=client.config_path("ball"))
    parser.add_argument("--node", action="append", default=[], metavar="WHEEL:ROLE@HOST[:PORT]")
    parser.add_argument("--discovery-address")
    parser.add_argument("--discovery-timeout", type=float, default=3.0)
    if command == "start":
        parser.add_argument("--bag", default="latest", help="active robot bag, default: latest hardware recorder")
    args = parser.parse_args(argv)
    try:
        if not 0 < args.discovery_timeout <= 60:
            raise ValueError("discovery-timeout must be positive and at most 60 seconds")
        root = args.root.expanduser().resolve()
        root.mkdir(parents=True, exist_ok=True)
        configuration = client.load_config(args.config)
        if configuration.get("caster_type", "ball") != "ball":
            raise ValueError("Use a ball-caster recording config")
        # Serialize start/stop across terminals, including latest-marker updates.
        with client.run_lock(root):
            return (start if command == "start" else stop)(args, root, configuration, client.Transport())
    except (OSError, ValueError, RuntimeError, KeyError, TypeError, http.client.HTTPException) as error:
        print(f"{command}_encoder_recording: {error}", file=sys.stderr)
        return 2

