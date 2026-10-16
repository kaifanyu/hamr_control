#!/usr/bin/env python3
"""Capture actual Gazebo terrain images and measure bounded contact/robot checks.

No trajectory animation is used. Guided drop fixtures test support height;
free spheres and sliding boxes receive finite physical wrench pulses. A separate
--caster run uses only the vehicle's existing rear-wheel velocity controllers.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np

CAMERAS = {"overview": "/hamr/terrain/overview/image", "detail": "/hamr/terrain/detail/image"}


def write_json(path, data):
    Path(path).write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")


def stamp(msg):
    return msg.sec + msg.nanosec * 1e-9


def support_height(grid, xy, radius):
    """Sphere support envelope over the exact a-b-c / a-c-d triangle grid.

    The dense search spacing is <= radius/80; a 1 mm allowance in the physical
    test exceeds its submillimeter discretization error. It is not h(x,y)+r,
    which is wrong on slopes and sharp bumps.
    """
    if "collision_vertices" in grid and "collision_faces" in grid:
        from generate_rocky_terrains import sphere_support
        return sphere_support(grid["collision_vertices"], grid["collision_faces"], xy, radius)
    gx, gy, gz = (np.asarray(grid[k]) for k in ("x", "y", "z"))
    offsets = np.linspace(-radius, radius, 161)
    xx, yy = np.meshgrid(xy[0]+offsets, xy[1]+offsets)
    rr = (xx-xy[0])**2 + (yy-xy[1])**2
    valid = rr <= radius**2
    ix = np.clip(np.searchsorted(gx, xx, side="right")-1, 0, len(gx)-2)
    iy = np.clip(np.searchsorted(gy, yy, side="right")-1, 0, len(gy)-2)
    u, v = (xx-gx[ix])/(gx[ix+1]-gx[ix]), (yy-gy[iy])/(gy[iy+1]-gy[iy])
    a, b, c, d = gz[iy,ix], gz[iy,ix+1], gz[iy+1,ix+1], gz[iy+1,ix]
    zz = np.where(u >= v, a+(b-a)*u+(c-b)*v, a+(c-d)*u+(d-a)*v)
    envelope = zz + np.sqrt(np.maximum(0., radius**2-rr))
    return float(np.max(envelope[valid]))


def probe_specs(metadata, grid):
    result = []
    for index, p in enumerate(metadata["probes"]):
        xy = list(p["xy_m"])
        support = support_height(grid, xy, .04)
        result.append({"name": "drop_"+p.get("name", str(index)), "kind": "guided_drop",
                       "xy_m": xy, "radius_m": .04, "support_z_m": support,
                       "spawn_z_m": support+.22, "mass_kg": .2, "friction": 1.})
    if metadata.get("flat_friction_probes", True) is False:
        # A slope cannot identify Coulomb friction via the flat-floor formula.
        return result
    # The metadata's flat approach is intentionally used for Coulomb tests;
    # rolling a sphere cannot identify sliding friction.
    flat_y = metadata.get("flat_probe_y_m", -3.5)
    for x, y, friction in ((-1.8, flat_y-.2, .3), (-1.8, flat_y+.2, 1.2)):
        result.append({"name": f"slide_mu_{str(friction).replace('.', '_')}", "kind": "sliding_box",
                       "xy_m": [x, y], "support_z_m": support_height(grid, [x,y], 0.),
                       "spawn_z_m": support_height(grid, [x,y], 0.)+.12,
                       "mass_kg": .2, "friction": friction,
                       "expected_combined_mu": min(1., friction)})
    xy = [-1.8, flat_y+.6]
    support = support_height(grid, xy, .04)
    result.append({"name": "free_rolling", "kind": "free_sphere", "xy_m": xy, "radius_m": .04,
                   "support_z_m": support, "spawn_z_m": support+.10, "mass_kg": .2, "friction": 1.})
    return result


def probe_sdf(spec):
    root = ET.Element("sdf", version="1.9")
    model = ET.SubElement(root, "model", name=spec["name"])
    x,y = spec["xy_m"]
    ET.SubElement(model,"pose").text = f"{x} {y} {spec['spawn_z_m']} 0 0 0"
    link = ET.SubElement(model,"link",name="body")
    inertial = ET.SubElement(link,"inertial")
    ET.SubElement(inertial,"mass").text = str(spec["mass_kg"])
    inertia = ET.SubElement(inertial,"inertia")
    moment = .4*spec["mass_kg"]*.04**2
    moments = {key: moment for key in ("ixx","iyy","izz")}
    if spec["kind"] == "sliding_box":
        moments = {"ixx": spec["mass_kg"]*(.06**2+.04**2)/12,
                   "iyy": spec["mass_kg"]*(.06**2+.04**2)/12,
                   "izz": spec["mass_kg"]*(.06**2+.06**2)/12}
    for key in ("ixx","iyy","izz","ixy","ixz","iyz"):
        ET.SubElement(inertia,key).text = str(moments.get(key, 0.))
    for role in ("collision","visual"):
        element = ET.SubElement(link,role,name="shape")
        geometry = ET.SubElement(element,"geometry")
        if spec["kind"] == "sliding_box":
            ET.SubElement(ET.SubElement(geometry,"box"),"size").text = ".06 .06 .04"
        else:
            ET.SubElement(ET.SubElement(geometry,"sphere"),"radius").text = ".04"
        if role == "collision":
            surface = ET.SubElement(element,"surface")
            friction = ET.SubElement(ET.SubElement(surface,"friction"),"ode")
            for tag in ("mu","mu2"):
                ET.SubElement(friction,tag).text = str(spec["friction"])
            ET.SubElement(ET.SubElement(surface,"bounce"),"restitution_coefficient").text="0"
        else:
            material = ET.SubElement(element,"material")
            for tag in ("ambient","diffuse"):
                ET.SubElement(material,tag).text = ".95 .22 .04 1"
    sensor = ET.SubElement(link,"sensor",name="contact",type="contact")
    ET.SubElement(sensor,"always_on").text="true"
    ET.SubElement(sensor,"update_rate").text="100"
    contact = ET.SubElement(sensor,"contact")
    ET.SubElement(contact,"collision").text="shape"
    ET.SubElement(contact,"topic").text=f"/hamr/terrain/probe/{spec['name']}/contacts"
    if spec["kind"] == "guided_drop":
        joint = ET.SubElement(model,"joint",name="vertical_guide",type="prismatic")
        ET.SubElement(joint,"parent").text="world"
        ET.SubElement(joint,"child").text="body"
        axis = ET.SubElement(joint,"axis")
        ET.SubElement(axis,"xyz").text="0 0 1"
        limit = ET.SubElement(axis,"limit")
        ET.SubElement(limit,"lower").text="-2"
        ET.SubElement(limit,"upper").text="2"
    plugin = ET.SubElement(model,"plugin",filename="gz-sim-odometry-publisher-system",
                           name="gz::sim::systems::OdometryPublisher")
    for tag,value in {"odom_frame":"world", "robot_base_frame":"body", "dimensions":"3",
                      "odom_publish_frequency":"100", "odom_topic":f"/hamr/terrain/probe/{spec['name']}/odom"}.items():
        ET.SubElement(plugin,tag).text=value
    return ET.tostring(root,encoding="unicode")


def prepare_world(source, output, rtf):
    tree = ET.parse(source)
    world = tree.getroot().find("world")
    if world is None:
        raise ValueError("Expected one named SDF world")
    # Copying a snapshot must not break original relative mesh references.
    for uri in world.findall(".//uri"):
        if uri.text and uri.text.startswith("model://hamr_terrain/"):
            uri.text = str((Path(source).parent.parent/uri.text.removeprefix("model://hamr_terrain/")).resolve())
        elif uri.text and not "://" in uri.text and not Path(uri.text).is_absolute():
            uri.text = str((Path(source).parent/uri.text).resolve())
    for short, name in (("contact","Contact"),("apply-link-wrench","ApplyLinkWrench")):
        filename=f"gz-sim-{short}-system"
        if not any(p.get("filename") == filename for p in world.findall("plugin")):
            ET.SubElement(world,"plugin",filename=filename,name=f"gz::sim::systems::{name}")
    if rtf is not None:
        physics=world.find("physics")
        if physics is None:
            raise ValueError("Source world must declare its physics settings")
        factor=physics.find("real_time_factor")
        if factor is None: factor=ET.SubElement(physics,"real_time_factor")
        factor.text=str(rtf)
    tree.write(output,encoding="unicode",xml_declaration=True)
    return world.get("name")


def stop_group(process):
    if process is None: return None
    result={"process_group":process.pid,"signals":[]}
    for sig, timeout in ((signal.SIGINT,8),(signal.SIGTERM,3),(signal.SIGKILL,1)):
        process.poll()
        try: os.killpg(process.pid,0)
        except ProcessLookupError: break
        os.killpg(process.pid,sig)
        result["signals"].append(sig.name)
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            process.poll()
            try: os.killpg(process.pid,0)
            except ProcessLookupError: break
            time.sleep(.05)
    process.poll()
    try: os.killpg(process.pid,0); present=True
    except ProcessLookupError: present=False
    result.update(leader_exit_code=process.returncode,group_present_after_cleanup=present)
    return result


def summarize_probe(spec, samples, contacts, pulse):
    result={"specification":spec,"sample_count":len(samples),"contact_messages":len(contacts)}
    if len(samples)<30:
        return dict(result,passed=False,error="Insufficient measured odometry")
    positions=np.array([s["position_m"] for s in samples])
    times=np.array([s["stamp_s"] for s in samples])
    tail=positions[times>=times[-1]-.7]
    depths=[d for c in contacts for d in c["depths_m"]]
    result.update(final_position_m=positions[-1].tolist(),tail_height_range_m=float(np.ptp(tail[:,2])),
                  measured_drop_m=float(positions[0,2]-np.median(tail[:,2])),
                  maximum_contact_depth_m=max(depths,default=None),
                  contact_collision_names=sorted({n for c in contacts for n in c["collision_names"]}))
    checks={"finite_feedback":bool(np.all(np.isfinite(positions))),
            "terrain_contact_observed":any("terrain" in n for n in result["contact_collision_names"]),
            "bounded_penetration":bool(depths and max(depths)<.012),
            "stayed_on_terrain_mesh":not any(n.startswith("apron_") for n in result["contact_collision_names"]),
            "no_other_probe_collision":not any((n.startswith("slide_") or n.startswith("free_rolling") or n.startswith("drop_")) and not n.startswith(spec.get("name", "")+"::") for n in result["contact_collision_names"])}
    if spec["kind"]=="guided_drop":
        error=float(np.median(tail[:,2])-spec["support_z_m"])
        result["support_height_error_m"]=error
        checks.update(physical_drop=result["measured_drop_m"]>.12,
                      support_height=abs(error)<.006,settled=result["tail_height_range_m"]<.003,
                      guide_xy_held=float(np.max(np.linalg.norm(positions[:,:2]-np.array(spec["xy_m"]),axis=1)))<.001)
    else:
        result["pulse"]=pulse
        checks["pulse_applied"]=bool(pulse and "stop_s" in pulse)
        displacement=float(np.linalg.norm(positions[-1,:2]-positions[0,:2]))
        result["horizontal_displacement_m"]=displacement
        if pulse and "stop_s" in pulse:
            coast_speed=np.array([math.hypot(*s["linear_velocity_m_s"][:2]) for s in samples])
            coast=(times>pulse["stop_s"]+.08)&(times<pulse["stop_s"]+.6)
            if np.count_nonzero(coast)>=5:
                acceleration=float(np.polyfit(times[coast],coast_speed[coast],1)[0])
                result["post_clear_coast_acceleration_m_s2"]=acceleration
                checks["force_off_observed"]=acceleration<.15
            else:
                checks["force_off_observed"]=False
        if spec["kind"]=="free_sphere":
            checks["free_motion"]=displacement>.015
            checks["supported_final_height"]=abs(float(np.median(tail[:,2]))-spec["support_z_m"])<.008
        elif pulse and "stop_s" in pulse:
            # Use measured velocity only after the wrench has been removed,
            # excluding its transport/update boundary and near-zero stiction.
            velocities=np.array([math.hypot(*s["linear_velocity_m_s"][:2]) for s in samples])
            mask=(times>pulse["stop_s"]+.025)&(velocities>.12)
            ids=np.flatnonzero(mask)
            if len(ids)>=5:
                slope,intercept=np.polyfit(times[ids]-times[ids][0],velocities[ids],1)
                estimate=float(-slope/9.80665)
                result.update(measured_sliding_mu=estimate,friction_fit_samples=len(ids),
                              friction_estimator={"method":"Linear fit of horizontal body-frame speed norm after wrench removal, divided by gravity; yaw invariant for the nearly level boxes",
                                  "gravity_m_s2":9.80665,"post_pulse_exclusion_s":.025,"minimum_speed_m_s":.12,
                                  "scope":"Longitudinal deceleration/g proxy and broad friction-response check; yaw/mesh/contact-solver coupling prevents precise coefficient calibration"},
                              friction_fit_duration_s=float(times[ids[-1]]-times[ids[0]]),
                              friction_fit_velocity_range_m_s=[float(velocities[ids[-1]]),float(velocities[ids[0]])])
                checks["sliding_friction_matches"]=abs(estimate-spec["expected_combined_mu"])<.18
            else:
                checks["sliding_friction_matches"]=False
                result["friction_fit_error"]="Insufficient post-pulse moving samples"
    result["checks"]=checks
    result["passed"]=all(checks.values())
    return result


def robot_summary(samples, requested_distance, contacts=None):
    if len(samples)<20: return {"passed":False,"error":"Insufficient robot feedback"}
    pos=np.array([s["position_m"] for s in samples])
    rpy=np.array([s["rpy_rad"] for s in samples])
    displacement=(pos[-1]-pos[0]).tolist()
    checks={"finite_feedback":bool(np.all(np.isfinite(pos)) and np.all(np.isfinite(rpy))),
            "forward_progress":displacement[1]>.55*requested_distance,
            "bounded_lateral_drift":abs(displacement[0])<.6,
            "upright":float(np.max(np.abs(rpy[:,:2])))<math.radians(35),
            "above_terrain_bottom":float(np.min(pos[:,2]))>-.12}
    contacts=contacts or []
    names=sorted({name for c in contacts for name in c["collision_names"]})
    depths=[d for c in contacts for d in c["depths_m"]]
    checks["robot_terrain_contact"]=any("compa::" in name for name in names) and any("terrain::" in name for name in names)
    rear_names=[name for name in names if any(part in ("left_wheel_link", "right_wheel_link") for part in name.split("::"))]
    front_names=[name for name in names if any(part.endswith(("_caster_wheel_link", "_hemisphere_link", "_polar_roller_link")) for part in name.split("::"))]
    checks["rear_wheel_contact_observed"]=bool(rear_names)
    checks["front_caster_contact_observed"]=bool(front_names)
    checks["bounded_contact_penetration"]=bool(depths and max(depths)<.015)
    return {"passed":all(checks.values()),"checks":checks,"displacement_m":displacement,
            "contact_collision_names":names,"rear_wheel_contact_names":rear_names,"front_rolling_contact_names":front_names,
            "other_caster_contact_names":[name for name in names if "caster" in name and name not in front_names],
            "contact_messages":len(contacts),"maximum_contact_depth_m":max(depths,default=None),
            "maximum_abs_roll_pitch_deg":np.degrees(np.max(np.abs(rpy[:,:2]),axis=0)).tolist(),
            "minimum_root_z_m":float(np.min(pos[:,2])),"sample_count":len(samples),
            "scope":"Short open-loop rear-wheel drive smoke check, not a path-tracking or traction calibration"}


def main(argv=None):
    def interrupted(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--world",required=True,type=Path)
    p.add_argument("--metadata",required=True,type=Path)
    p.add_argument("--output-dir",required=True,type=Path)
    p.add_argument("--domain-id",type=int,default=101)
    p.add_argument("--real-time-factor",type=float,default=.5)
    p.add_argument("--wall-timeout",type=float,default=300.)
    p.add_argument("--caster",choices=("none","ball","swivel"),default="none")
    p.add_argument("--robot-duration",type=float,default=10.)
    p.add_argument("--robot-speed",type=float,default=.12)
    p.add_argument("--prepare-only",action="store_true")
    p.add_argument("--render-backend",choices=("egl","display","d3d12"),default="egl")
    args=p.parse_args(argv)
    output=args.output_dir.expanduser().resolve()
    if output.exists() and any(output.iterdir()): p.error("Output directory must be new or empty")
    output.mkdir(parents=True,exist_ok=True)
    metadata=json.loads(args.metadata.read_text())
    grid=np.load((args.metadata.parent/metadata["height_grid"]).resolve())
    specs=probe_specs(metadata,grid) if args.caster=="none" else []
    world=prepare_world(args.world,output/"world.sdf",args.real_time_factor)
    for spec in specs: (output/f"{spec['name']}.sdf").write_text(probe_sdf(spec))
    write_json(output/"metadata.json",metadata)
    partition=f"hamr_terrain_check_{args.domain_id}_{os.getpid()}"
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id),ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST",GZ_PARTITION=partition)
    if args.render_backend=="d3d12":
        os.environ.update(GALLIUM_DRIVER="d3d12",MESA_D3D12_DEFAULT_ADAPTER_NAME="NVIDIA")
    command=["gz","sim","-s","-r"]+(["--headless-rendering"] if args.render_backend=="egl" else [])+["-v","3",str(output/"world.sdf")]
    if args.caster!="none":
        smoke=metadata.get("smoke", {})
        spawn={"x":smoke.get("spawn_x",metadata["spawn"]["x"]), "y":smoke.get("spawn_y",metadata["spawn"]["y"]),
               "z":smoke.get("spawn_z",metadata["spawn"]["z"]), "yaw":smoke.get("initial_yaw",metadata["spawn"]["yaw"])}
        command=["ros2","launch","hamr_terrain","terrain_sim.launch.py",f"world:={output/'world.sdf'}",
                 f"caster:={args.caster}","gui:=false",f"spawn_x:={spawn['x']}",f"spawn_y:={spawn['y']}",
                 f"spawn_z:={spawn['z']}",f"initial_yaw:={spawn['yaw']}",
                 f"headless_rendering:={str(args.render_backend=='egl').lower()}"]
    provenance={"created_utc":datetime.now(timezone.utc).isoformat(),"source_world":str(args.world.resolve()),
                "source_world_sha256":hashlib.sha256(args.world.read_bytes()).hexdigest(),
                "snapshot_sha256":hashlib.sha256((output/"world.sdf").read_bytes()).hexdigest(),
                "world_name":world,"ros_domain_id":args.domain_id,"gz_partition":partition,
                "command":command,"caster":args.caster,"real_time_factor_override":args.real_time_factor,
                "physics_changes":"Added Contact/ApplyLinkWrench systems; terrain collision, friction, inertia and timestep unchanged",
                "probe_specifications":specs}
    write_json(output/"commands.json",provenance)
    if args.prepare_only: return 0
    import rclpy
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image
    from nav_msgs.msg import Odometry
    from ros_gz_interfaces.msg import Contacts, Entity, EntityWrench
    from std_msgs.msg import Float64
    from PIL import Image as PILImage
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node("hamr_terrain_physical_check")
    data={"clock":None,"images":{},"samples":{s["name"]:[] for s in specs},
          "contacts":dict({s["name"]:[] for s in specs},terrain=[]),"robot":[],"pulses":{},"image_stage":"","phase":"waiting_for_cameras"}
    node.create_subscription(Clock,"/clock",lambda msg:data.__setitem__("clock",stamp(msg.clock)),10)
    def image_callback(msg,name):
        name=data["image_stage"]+name
        if name in data["images"]: return
        if msg.encoding not in ("rgb8","bgr8"): return
        raw=np.frombuffer(bytes(msg.data),dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
        if msg.encoding=="bgr8": raw=raw[:,:,::-1]
        PILImage.fromarray(raw).save(output/f"{name}.png")
        data["images"][name]={"file":f"{name}.png","source_stamp_s":stamp(msg.header.stamp),"width":msg.width,"height":msg.height,
                              "sha256":hashlib.sha256((output/f"{name}.png").read_bytes()).hexdigest(),"source":"Actual Gazebo camera sensor"}
    qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE)
    for name,topic in CAMERAS.items(): node.create_subscription(Image,topic,lambda msg,n=name:image_callback(msg,n),qos)
    def odometry(msg,name):
        q=msg.pose.pose.orientation
        roll=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))
        pitch=math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x))))
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        a=msg.pose.pose.position; v=msg.twist.twist.linear; w=msg.twist.twist.angular
        sample={"stamp_s":stamp(msg.header.stamp),"position_m":[a.x,a.y,a.z],"rpy_rad":[roll,pitch,yaw],
                "linear_velocity_m_s":[v.x,v.y,v.z],"angular_velocity_rad_s":[w.x,w.y,w.z]}
        (data["robot"] if name=="robot" else data["samples"][name]).append(sample)
    def contacts(msg,name):
        if not msg.contacts: return
        data["contacts"][name].append({"stamp_s":stamp(msg.header.stamp),
            "collision_names":[n.name for c in msg.contacts for n in (c.collision1,c.collision2)],
            "depths_m":[float(d) for c in msg.contacts for d in c.depths]})
    bridge=["ros2","run","ros_gz_bridge","parameter_bridge"]
    if args.caster=="none": bridge.append("/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock")
    bridge += [f"{topic}@sensor_msgs/msg/Image[gz.msgs.Image" for topic in CAMERAS.values()]
    force_publisher=clear_publisher=None
    if specs:
        force_topic=f"/world/{world}/wrench/persistent"
        clear_topic=f"/world/{world}/wrench/clear"
        force_publisher=node.create_publisher(EntityWrench,force_topic,10)
        clear_publisher=node.create_publisher(Entity,clear_topic,10)
        bridge += [force_topic+"@ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench",
                   clear_topic+"@ros_gz_interfaces/msg/Entity]gz.msgs.Entity"]
    for spec in specs:
        name=spec["name"]; prefix=f"/hamr/terrain/probe/{name}"
        node.create_subscription(Odometry,prefix+"/odom",lambda msg,n=name:odometry(msg,n),100)
        node.create_subscription(Contacts,prefix+"/contacts",lambda msg,n=name:contacts(msg,n),100)
        bridge += [prefix+"/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                   prefix+"/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts"]
    publishers=[]
    if args.caster!="none":
        node.create_subscription(Contacts,"/hamr/terrain/contacts",lambda msg:contacts(msg,"terrain"),100)
        bridge.append("/hamr/terrain/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts")
        node.create_subscription(Odometry,"/compa/odom",lambda msg:odometry(msg,"robot"),100)
        publishers=[node.create_publisher(Float64,f"/{side}_wheel/cmd_vel",10) for side in ("left","right")]
    started_wall=time.monotonic()
    deadline=started_wall+args.wall_timeout
    next_progress=started_wall
    def progress(force=False):
        nonlocal next_progress
        now=time.monotonic()
        if not force and now<next_progress:return
        next_progress=now+5.
        temporary=output/"progress.json.tmp"
        write_json(temporary,{"phase":data["phase"],"sim_time_s":data["clock"],
                              "wall_elapsed_s":now-started_wall,"wall_timeout_s":args.wall_timeout,
                              "images_received":sorted(data["images"]),"robot_samples":len(data["robot"]),
                              "probe_samples":{name:len(samples) for name,samples in data["samples"].items()}})
        temporary.replace(output/"progress.json")
    def spin_until(predicate):
        while not predicate():
            if time.monotonic()>deadline: raise TimeoutError("Terrain check exceeded wall timeout")
            if simulation.poll() is not None: raise RuntimeError("Gazebo exited before check completion")
            rclpy.spin_once(node,timeout_sec=.02)
            progress()
    def gz_service(action,reqtype,request):
        result=subprocess.run(["gz","service","-s",f"/world/{world}/{action}","--reqtype",reqtype,"--reptype","gz.msgs.Boolean",
                               "--timeout","5000","--req",request],text=True,capture_output=True,timeout=10)
        if result.returncode or "true" not in result.stdout: raise RuntimeError(f"{action}: {result.stdout} {result.stderr}")
    def pause_and_drain():
        gz_service("control","gz.msgs.WorldControl","pause: true")
        until=time.monotonic()+.2
        while time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=.002)

    def wrench(name,force):
        # Keep transport publishers alive across the entire experiment. A
        # one-shot CLI publisher can exit before discovery/delivery completes.
        entity=Entity(name=name+"::body",type=Entity.LINK)
        if force:
            message=EntityWrench(entity=entity)
            message.wrench.force.x=force
            force_publisher.publish(message)
        else:
            # Clearing is idempotent. Repeat while simulation is paused so no
            # simulated-time impulse is hidden by transport retries.
            for _ in range(3):
                clear_publisher.publish(entity)
                until=time.monotonic()+.10
                while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.005)
        until=time.monotonic()+.2
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.005)
    simulation=bridge_process=None; handles=[]; report={"provenance":provenance,"passed":False}
    try:
        for filename,cmd in (("gazebo.log",command),("bridge.log",bridge)):
            handle=open(output/filename,"w"); handles.append(handle)
            proc=subprocess.Popen(cmd,stdout=handle,stderr=subprocess.STDOUT,start_new_session=True)
            if filename=="gazebo.log": simulation=proc
            else: bridge_process=proc
        spin_until(lambda:data["clock"] is not None and len(data["images"])==len(CAMERAS) and (args.caster=="none" or bool(data["robot"])))
        report["images"]=data["images"]
        if specs:
            spin_until(lambda:force_publisher.get_subscription_count()>0 and clear_publisher.get_subscription_count()>0)
            data["phase"]="spawning_probes"
            gz_service("control","gz.msgs.WorldControl","pause: true")
            for spec in specs:
                gz_service("create","gz.msgs.EntityFactory",f'sdf_filename: "{output/spec["name"]}.sdf" allow_renaming: false')
            gz_service("control","gz.msgs.WorldControl","pause: false")
            spin_until(lambda:all(data["samples"][s["name"]] for s in specs))
            started=data["clock"]
            data["phase"]="settling"
            spin_until(lambda:data["clock"]>=started+2.)
            for spec in specs:
                if spec["kind"]=="guided_drop": continue
                name=spec["name"]
                data["phase"]="wrench_"+name
                force=(2. if spec["friction"]<1. else 4.) if spec["kind"]=="sliding_box" else .3
                pause_and_drain()
                wrench(name,force)
                pulse=data["pulses"][name]={"force_N":force,"start_s":data["clock"],"timing":"World paused while configuring and clearing wrench; source clock measured at each boundary",
                    "delivery":"Persistent ROS-to-Gazebo bridge publishers; one force message and three idempotent clear messages while paused"}
                gz_service("control","gz.msgs.WorldControl","pause: false")
                spin_until(lambda:data["clock"]>=pulse["start_s"]+.20)
                pause_and_drain()
                wrench(name,0.)
                pulse["stop_s"]=data["clock"]
                data["phase"]="coast_"+name
                gz_service("control","gz.msgs.WorldControl","pause: false")
                spin_until(lambda:data["clock"]>=pulse["stop_s"]+1.2)
            ended=data["clock"]+1.
            spin_until(lambda:data["clock"]>=ended)
            report["probes"]={s["name"]:summarize_probe(s,data["samples"][s["name"]],data["contacts"][s["name"]],data["pulses"].get(s["name"])) for s in specs}
            report["passed"]=all(p["passed"] for p in report["probes"].values())
        else:
            started=data["clock"]
            data["phase"]="settling"
            spin_until(lambda:data["clock"]>=started+2.)
            motion_start=data["clock"]
            data["phase"]="driving"
            while data["clock"]<motion_start+args.robot_duration:
                for pub in publishers: pub.publish(Float64(data=args.robot_speed/.1075))
                next_command=min(motion_start+args.robot_duration,data["clock"]+.02)
                spin_until(lambda:data["clock"]>=next_command)
            for pub in publishers: pub.publish(Float64(data=0.))
            data["phase"]="final_settle"
            ended=data["clock"]+1.
            spin_until(lambda:data["clock"]>=ended)
            data["phase"]="final_images"
            data["image_stage"]="final_"
            spin_until(lambda:all("final_"+name in data["images"] for name in CAMERAS))
            report["robot"]=robot_summary(data["robot"],args.robot_duration*args.robot_speed,data["contacts"]["terrain"])
            report["passed"]=report["robot"]["passed"]
    except (Exception,KeyboardInterrupt) as error:
        report["error"]=f"{type(error).__name__}: {error}"
    finally:
        data["phase"]="cleanup"
        progress(force=True)
        for pub in publishers: pub.publish(Float64(data=0.))
        report["cleanup"]={"simulation":stop_group(simulation),"bridge":stop_group(bridge_process)}
        for handle in handles: handle.close()
        report["images"]=data["images"]
        report["sim_end_s"]=data["clock"]
        report["measurement_scope"]="Measured Gazebo DART contact response; prescribed wheel motors/wrenches only. Not hardware calibration. Guided drop fixtures intentionally suppress horizontal motion."
        data["phase"]="complete" if report["passed"] else "failed"
        progress(force=True)
        write_json(output/"measurements.json",data)
        write_json(output/"report.json",report)
        node.destroy_node(); rclpy.shutdown()
    print(json.dumps({"output_dir":str(output),"passed":report["passed"],"error":report.get("error")}))
    return 0 if report["passed"] else 1


if __name__=="__main__":
    raise SystemExit(main())
