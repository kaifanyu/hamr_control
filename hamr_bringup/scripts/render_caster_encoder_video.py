#!/usr/bin/env python3
"""Render recorded caster encoder packets to a timestamped, constant-rate CAD MP4.

No ROS playback process or physics solver is needed. Each video frame uses the
last valid measured pose at its bag timestamp, through the same ReplayModel as
RViz. Rendering wall time cannot change video speed.
"""
from __future__ import annotations

import argparse
from collections import Counter
import csv
from fractions import Fraction
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np

from caster_replay_model import DEFAULT_MAPPING, ReplayModel, validate_playback
from caster_encoder_protocol import decode_packet, parse_json
from caster_encoder_selection import WHEELS, WheelIdentityError, WheelIdentityGuard, resolve_selection

TOPIC = '/caster_encoder/packets'
HIDDEN_MESHES = {'skate_bearing.stl', '4668k11_permanently_lubricated_stainless_steel_ball_bearing.stl',
                 '5905k22_needle_roller_bearing.stl', '5_16_18screw.stl', '5_16nuts.stl'}


def frame_times(start_ns, end_ns, fps, rate):
    """Integer timestamps on the output clock, with <1 frame of duration padding."""
    if not 1 <= fps <= 120 or not math.isfinite(rate) or rate <= 0 or end_ns <= start_ns:
        raise ValueError('Require fps 1..120, positive finite rate, and a nonempty bag duration')
    step = Fraction(str(rate)) * 1_000_000_000 / fps
    count = math.ceil(Fraction(end_ns - start_ns, 1) / step)
    return [start_ns + int(i * step) for i in range(count)]


def read_bag(path, topic=None, wheel_id=None):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import String

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=''), rosbag2_py.ConverterOptions('', ''))
    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wheel_id, selected_topic = resolve_selection(wheel_id, topic, topics)
    guard = WheelIdentityGuard(wheel_id)
    if topics.get(selected_topic) != 'std_msgs/msg/String':
        raise ValueError(f'{path} does not contain {selected_topic} as std_msgs/msg/String')
    events, start, end = [], None, None
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        start = stamp if start is None else min(start, stamp)
        end = stamp if end is None else max(end, stamp)
        if topic == selected_topic:
            message = deserialize_message(data, String).data
            try:
                envelope = parse_json(message)
                packet = decode_packet(json.dumps(envelope['packet'], allow_nan=False).encode('utf-8'))
            except (ValueError, KeyError, TypeError, OverflowError, RecursionError):
                # The replay model counts malformed records and holds the last
                # valid pose. Identity conflicts must instead stop the render.
                pass
            else:
                guard.accept(packet)
            events.append((stamp, message))
    del reader
    if not events or start == end:
        raise ValueError('Bag contains no replayable encoder interval')
    events.sort(key=lambda item: item[0])
    return events, start, end


def sample_frames(events, times, mapping=None, stale_timeout_s=1., wheel_id=None):
    model = ReplayModel(mapping, stale_timeout_s, wheel_id)
    frames, index = [], 0
    for stamp in times:
        while index < len(events) and events[index][0] <= stamp:
            received, envelope = events[index]
            try:
                model.receive(envelope, received)
            except WheelIdentityError:
                raise
            except ValueError:
                pass  # The shared model counts/reports rejects and holds valid data.
            index += 1
        names, positions, status = model.snapshot(stamp)
        frames.append({'stamp_ns': stamp, 'joints': dict(zip(names, positions)), 'status': status})
    if not any(frame['status']['complete_pose_available'] for frame in frames):
        raise ValueError('No complete H1/H2/CENTER pose is available at the requested frame times')
    return frames


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    cross = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + math.sin(angle) * cross + (1 - math.cos(angle)) * (cross @ cross)


def origin(element):
    result = np.eye(4)
    if element is not None:
        result[:3, 3] = np.fromstring(element.get('xyz', '0 0 0'), sep=' ')
        r, p, y = np.fromstring(element.get('rpy', '0 0 0'), sep=' ')
        result[:3, :3] = rotation([0, 0, 1], y) @ rotation([0, 1, 0], p) @ rotation([1, 0, 0], r)
    return result


def link_transforms(robot, angles):
    pending = list(robot.findall('joint'))
    roots = {l.get('name') for l in robot.findall('link')} - {j.find('child').get('link') for j in pending}
    if len(roots) != 1:
        raise ValueError('CAD URDF must have one root')
    transforms = {roots.pop(): np.eye(4)}
    while pending:
        resolved = []
        for joint in pending:
            parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
            if parent not in transforms:
                continue
            motion = np.eye(4)
            if joint.get('type') in ('continuous', 'revolute'):
                axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                motion[:3, :3] = rotation(axis, angles.get(joint.get('name'), 0.))
            elif joint.get('type') != 'fixed':
                raise ValueError('This renderer supports fixed and rotational caster joints only')
            transforms[child] = transforms[parent] @ origin(joint.find('origin')) @ motion
            resolved.append(joint)
        if not resolved:
            raise ValueError('CAD URDF joints are disconnected or cyclic')
        pending = [j for j in pending if j not in resolved]
    return transforms


def ground_start_mapping(events, robot, mapping=None, stale_timeout_s=1., wheel_id=None):
    """Assume a physical reference pose once; retain all subsequent angle changes."""
    model = ReplayModel(mapping, stale_timeout_s, wheel_id)
    reference = None
    for stamp, envelope in events:
        try:
            model.receive(envelope, stamp)
        except WheelIdentityError:
            raise
        except ValueError:
            continue
        status = model.snapshot(stamp)[2]
        if status['complete_pose_available'] and all(n['display_state'] == 'fresh' for n in status['nodes']):
            reference = {role: model.values[node]['source_angle_rad'] for node, role in ((1, 'h1'), (2, 'h2'), (3, 'center'))}
            reference_stamp = stamp
            break
    if reference is None:
        raise ValueError('Ground-contact start needs a complete fresh H1/H2/CENTER reference pose')
    effective = dict(model.mapping)
    transforms = link_transforms(robot, {})
    carrier = robot.find("./joint[@name='caster_carrier_joint']")
    shell = robot.find(f"./joint[@name='{effective['h1_joint']}']")
    if carrier is None or shell is None:
        raise ValueError('Ground-contact start requires the standalone caster CAD joints')
    carrier_axis = transforms[carrier.find('child').get('link')][:3, :3] @ np.fromstring(carrier.find('axis').get('xyz'), sep=' ')
    shell_axis = transforms[shell.find('child').get('link')][:3, :3] @ np.fromstring(shell.find('axis').get('xyz'), sep=' ')
    if not np.allclose(carrier_axis, [0, -1, 0], atol=1e-8) or abs(shell_axis[1]) > 1e-8:
        raise ValueError('Ground-contact reference requires the upright fork with a -Y carrier axle')
    # Rotate H1's outward split axis onto +X (forward), leaving H2 at -X.
    # Zero shell spin is a chosen visual phase, not a measured hole/index zero.
    targets = {'h1': 0., 'h2': 0., 'center': math.atan2(-shell_axis[2], shell_axis[0]) % (2 * math.pi)}
    for role in targets:
        effective[f'{role}_zero_rad'] = reference[role] - targets[role] / effective[f'{role}_sign']
    effective['mapping_verified'] = False
    return effective, {
        'mode': 'ground-contact', 'assumed': True, 'reference_stamp_ns': reference_stamp,
        'reference_encoder_rad': reference, 'target_model_rad': targets,
        'reference_devices': {role: {key: model.values[node][key] for key in ('wheel_id', 'device_id', 'boot_id', 'seq')}
                              for node, role in ((1, 'H1'), (2, 'H2'), (3, 'CENTER'))},
        'note': 'Assumed initial pose: upright fork, H1 forward (+X), H2 back (-X), split axis horizontal. '
                'First complete fresh encoder pose is the reference; subsequent signed angle changes are retained. '
                'No robot yaw, contact dynamics, or physical phase calibration is inferred.',
    }


def package_share(name):
    source = Path(__file__).resolve().parents[2] / name
    if (source / 'package.xml').is_file():
        return source
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory(name))


class Scene:
    def __init__(self, model_path, width, height, full_detail=False, ground_pose=None):
        import vtk
        from vtk.util.numpy_support import vtk_to_numpy
        self.vtk, self.to_numpy = vtk, vtk_to_numpy
        self.robot = ET.parse(model_path).getroot()
        self.width, self.height = width, height
        self.window = vtk.vtkRenderWindow()
        self.window.SetOffScreenRendering(1)
        self.window.SetSize(width, height)
        # Some onboard GPU drivers cannot compile VTK's MSAA resolve shader.
        # FXAA smooths the rendered edges without requiring that shader.
        self.window.SetMultiSamples(0)
        background = vtk.vtkRenderer()
        background.SetBackground(.925, .945, .965)
        self.window.AddRenderer(background)
        self.actors, self.meshes = [], {}
        self.renderers = []
        for viewport, position, target, scale in [
            ((0., .12, .68, .81), (.52, -.75, .40), (0., 0., .05), .19),
            ((.70, .49, .985, .80), (-.48, .66, .23), (0., 0., .025), .165),
        ]:
            renderer = vtk.vtkRenderer()
            renderer.UseFXAAOn()
            renderer.SetViewport(*viewport)
            renderer.SetBackground(.925, .945, .965)
            self.window.AddRenderer(renderer)
            camera = renderer.GetActiveCamera()
            camera.SetPosition(*position); camera.SetFocalPoint(*target); camera.SetViewUp(0, 0, 1)
            camera.ParallelProjectionOn(); camera.SetParallelScale(scale)
            camera.SetClippingRange(.01, 10.)
            for loc, intensity in [((1, -2, 3), .85), ((-1, 1, 2), .65)]:
                light = vtk.vtkLight(); light.SetPosition(*loc); light.SetFocalPoint(0, 0, 0)
                light.SetIntensity(intensity); renderer.AddLight(light)
            self.renderers.append(renderer)
        for link in self.robot.findall('link'):
            for visual in link.findall('visual'):
                mesh = visual.find('geometry/mesh')
                if mesh is None:
                    raise ValueError('Expected CAD mesh visuals')
                uri = mesh.get('filename')
                if not full_detail and Path(uri).name in HIDDEN_MESHES:
                    continue
                package, relative = uri.removeprefix('package://').split('/', 1)
                path = package_share(package) / relative
                if path not in self.meshes:
                    if not path.is_file():
                        raise ValueError(f'Missing CAD mesh: {path}')
                    reader = vtk.vtkSTLReader(); reader.SetFileName(str(path)); reader.Update()
                    if reader.GetOutput().GetNumberOfPoints() == 0:
                        raise ValueError(f'Empty CAD mesh: {path}')
                    normals = vtk.vtkPolyDataNormals()
                    normals.SetInputData(reader.GetOutput()); normals.SetFeatureAngle(30)
                    normals.SplittingOn(); normals.ConsistencyOn(); normals.Update()
                    mapper = vtk.vtkPolyDataMapper(); mapper.SetInputData(normals.GetOutput())
                    self.meshes[path] = mapper
                item = vtk.vtkActor(); item.SetMapper(self.meshes[path])
                rgba = np.fromstring(visual.find('material/color').get('rgba'), sep=' ')
                item.GetProperty().SetColor(*rgba[:3]); item.GetProperty().SetOpacity(rgba[3])
                item.GetProperty().SetAmbient(.23); item.GetProperty().SetDiffuse(.75)
                item.GetProperty().SetSpecular(.35); item.GetProperty().SetSpecularPower(45)
                transform = origin(visual.find('origin'))
                transform[:3, :3] = transform[:3, :3] @ np.diag(np.fromstring(mesh.get('scale', '1 1 1'), sep=' '))
                self.add(item, link.get('name'), transform)
        for link, point in [
            ('caster_positive_hemisphere_link', (-.0226, 0., .0974)),
            ('caster_negative_hemisphere_link', (.0927, 0., .0376)),
        ]:
            sphere = vtk.vtkSphereSource(); sphere.SetRadius(.0055)
            sphere.SetThetaResolution(24); sphere.SetPhiResolution(16); sphere.SetCenter(*point)
            mapper = vtk.vtkPolyDataMapper(); mapper.SetInputConnection(sphere.GetOutputPort())
            item = vtk.vtkActor(); item.SetMapper(mapper); item.GetProperty().SetColor(1., 1., .96)
            self.add(item, link, np.eye(4))
        self.ground_z = None
        if ground_pose is not None:
            # The floor is fixed at the reference shell height. It provides
            # context, never moves to force contact during measured replay.
            transforms = link_transforms(self.robot, ground_pose)
            bottoms = []
            for actor, link, local in self.actors:
                if link not in ('caster_positive_hemisphere_link', 'caster_negative_hemisphere_link'):
                    continue
                actor.GetMapper().Update()
                points = self.to_numpy(actor.GetMapper().GetInput().GetPoints().GetData())
                pose = transforms[link] @ local
                bottoms.append(float(np.min(points @ pose[2, :3] + pose[2, 3])))
            self.ground_z = min(bottoms)
            plane = vtk.vtkPlaneSource()
            plane.SetOrigin(-.24, -.24, self.ground_z)
            plane.SetPoint1(.24, -.24, self.ground_z)
            plane.SetPoint2(-.24, .24, self.ground_z)
            mapper = vtk.vtkPolyDataMapper(); mapper.SetInputConnection(plane.GetOutputPort())
            floor = vtk.vtkActor(); floor.SetMapper(mapper)
            floor.GetProperty().SetColor(.70, .74, .77)
            for renderer in self.renderers:
                renderer.AddActor(floor)
            for coordinate in np.arange(-.20, .201, .05):
                for a, b in [((coordinate, -.24), (coordinate, .24)), ((-.24, coordinate), (.24, coordinate))]:
                    line = vtk.vtkLineSource()
                    line.SetPoint1(*a, self.ground_z + .0001); line.SetPoint2(*b, self.ground_z + .0001)
                    mapper = vtk.vtkPolyDataMapper(); mapper.SetInputConnection(line.GetOutputPort())
                    grid = vtk.vtkActor(); grid.SetMapper(mapper); grid.GetProperty().SetColor(.48, .53, .58)
                    for renderer in self.renderers:
                        renderer.AddActor(grid)
        self.capture = vtk.vtkWindowToImageFilter(); self.capture.SetInput(self.window)
        self.capture.SetInputBufferTypeToRGB(); self.capture.ReadFrontBufferOff()

    def add(self, actor, link, visual_transform):
        self.actors.append((actor, link, visual_transform))
        for renderer in self.renderers:
            renderer.AddActor(actor)

    def render(self, frame):
        from PIL import Image
        transforms = link_transforms(self.robot, frame['joints'])
        complete = frame['status']['complete_pose_available']
        for actor, link, local in self.actors:
            actor.SetVisibility(complete or link == 'caster_fork_link')
            matrix = self.vtk.vtkMatrix4x4()
            pose = transforms[link] @ local
            for i in range(4):
                for j in range(4):
                    matrix.SetElement(i, j, pose[i, j])
            actor.SetUserMatrix(matrix)
        self.window.Render(); self.capture.Modified(); self.capture.Update()
        pixels = self.to_numpy(self.capture.GetOutput().GetPointData().GetScalars())
        return Image.fromarray(np.flipud(pixels.reshape(self.height, self.width, 3)).copy())


class Overlay:
    def __init__(self, width, bag_name, start, end, rate):
        from PIL import ImageFont
        self.scale = width / 1600
        self.bag, self.start, self.duration, self.rate = bag_name, start, (end-start)/1e9, rate
        fontroot = Path('/usr/share/fonts/truetype/dejavu')
        self.fonts = {name: ImageFont.truetype(str(fontroot/file), round(size*self.scale))
                      for name, file, size in [('title','DejaVuSans-Bold.ttf',34),
                                               ('label','DejaVuSans-Bold.ttf',20),
                                               ('text','DejaVuSans.ttf',20),
                                               ('small','DejaVuSans.ttf',17),
                                               ('value','DejaVuSansMono.ttf',26)]}

    def draw(self, image, frame):
        from PIL import ImageDraw
        draw, s = ImageDraw.Draw(image), self.scale
        def box(coords, fill): draw.rectangle(tuple(x*s for x in coords), fill=fill)
        def text(x, y, value, font='text', fill='#21334a'):
            draw.text((x*s,y*s),value,font=self.fonts[font],fill=fill)
        box((0,0,1600,126),'#12263b'); box((0,774,1600,900),'#12263b')
        text(38,24,'Ball caster | recorded encoder replay','title','#f7fbff')
        text(40,76,self.bag,'text','#c5d4e2')
        elapsed = (frame['stamp_ns']-self.start)/1e9
        text(1265,25,f'{self.rate:g}x  |  {elapsed:05.2f} s','value','#90e2cc')
        text(1265,75,f'Recorded span {self.duration:.2f} s','small','#c5d4e2')
        text(38,139,'CAD ASSEMBLY','label'); text(1134,139,'OPPOSITE VIEW','label')
        box((1106,474,1600,774),'#f6f8fb')
        text(1134,488,'REPLAY JOINT ANGLES','label')
        mapping=frame['status']['mapping']
        for i,node in enumerate(frame['status']['nodes']):
            y=530+i*71; role=node['role']; value=node['model_position_rad']
            color='#209d94' if role=='CENTER' else ('#2879d8' if mapping[role.lower()+'_joint']=='caster_positive_hemisphere_joint' else '#d36b1f')
            text(1134,y,role,'label',color)
            text(1260,y,'waiting' if value is None else f'{value:+8.3f} rad','value')
            text(1262,y+32,node['display_state'].replace('_',' '),'small','#687b90')
        if not frame['status']['complete_pose_available']:
            text(180,698,'Waiting for H1 + H2 + CENTER','label')
        calibrated=frame['status']['mapping_calibration_current']
        if frame['status'].get('start_pose', {}).get('assumed'):
            text(40,795,'Assumed start: H1 forward (+X), H2 back (-X), both at ground  |  Recorded angle changes', 'small','#d9e5ef')
        else:
            text(40,795,'Measured-motion visualization  |  CAD mapping '+('user-verified' if calibrated else 'illustrative: signs / zeros unverified'), 'small','#d9e5ef')
        text(40,824,'White dots: visual phase marks  |  Polar rollers: unmeasured, held at zero  |  Bag timing; no interpolation','small','#aebfd0')
        box((40,873,1560,879),'#344c63')
        box((40,873,40+1520*min(1,elapsed/self.duration),879),'#90e2cc')
        return image


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream,'sha256').hexdigest()


def render(args):
    bag, rate = validate_playback(args.bag,args.rate); bag=Path(bag)
    provenance_path = bag / 'local-recording-replay.json' if bag.is_dir() else bag.parent / 'local-recording-replay.json'
    local_recording = json.loads(provenance_path.read_text()) if provenance_path.is_file() else None
    if args.width < 800 or args.width % 32:
        raise ValueError('width must be >=800 and divisible by 32 for even 16:9 output')
    mapping, stale = None, 1.
    if args.mapping_config:
        import yaml
        params=yaml.safe_load(args.mapping_config.read_text())['replay_caster_encoder_joints']['ros__parameters']
        mapping={key:params[key] for key in DEFAULT_MAPPING if key in params}
        stale=params.get('stale_timeout_s',1.)
    wheel_id, topic = resolve_selection(args.wheel, args.topic)
    events,start,end=read_bag(bag, args.topic, args.wheel)
    model=package_share('hamr_ball_caster')/'urdf/ball_caster.urdf'
    start_pose = {'mode': 'recorded', 'assumed': False}
    ground_pose = None
    if getattr(args, 'start_pose', 'recorded') == 'ground-contact':
        mapping, start_pose = ground_start_mapping(events, ET.parse(model).getroot(), mapping, stale, wheel_id)
        ground_pose = {mapping['h1_joint']: 0., mapping['h2_joint']: 0.,
                       'caster_carrier_joint': start_pose['target_model_rad']['center']}
    times=frame_times(start,end,args.fps,rate)
    frames=sample_frames(events,times,mapping,stale,wheel_id)
    for frame in frames:
        frame['status']['start_pose'] = start_pose
    wheel_id = frames[-1]["status"]["wheel_id"]
    _, topic = resolve_selection(wheel_id, args.topic)
    output=args.output_dir.expanduser().resolve(); output.mkdir(parents=True,exist_ok=True)
    suffix='_plots' if args.plots else ''
    video=output/f'{bag.stem}_{wheel_id}_{rate:g}x{suffix}.mp4'
    if video.exists() and args.preview_time is None:
        raise ValueError(f'Refusing to overwrite {video}; choose a new output directory')
    scene=Scene(model,args.width,args.width*9//16,args.full_detail,ground_pose)
    overlay=Overlay(args.width,f"{bag.stem} / {wheel_id}",start,end,rate)
    plots=None
    if args.plots:
        from caster_encoder_plots import EncoderPlots
        plots=EncoderPlots(events,start,end,f"{bag.stem} / {wheel_id}",args.width,stale,wheel_id)
        if local_recording:
            plots.metadata['time_basis'] = local_recording['time_basis']
    def draw_frame(frame):
        image=overlay.draw(scene.render(frame),frame)
        return plots.draw(image,frame) if plots else image
    try:
        if args.preview_time is not None:
            if not 0 <= args.preview_time <= (end-start)/1e9:
                raise ValueError('preview-time must lie inside the bag interval')
            frame=frames[min(len(frames)-1,int(args.preview_time*args.fps/rate))]
            path=output/f'{bag.stem}_{wheel_id}{suffix}_preview.png'
            draw_frame(frame).save(path)
            path.with_suffix('.json').write_text(json.dumps({
                'start_pose': start_pose, 'mapping': frames[-1]['status']['mapping'],
                'ground_z_m': scene.ground_z, 'local_recording': local_recording,
                'preview_stamp_ns': frame['stamp_ns']}, indent=2) + '\n')
            print(path,flush=True)
            return
        ffmpeg=shutil.which('ffmpeg')
        if not ffmpeg: raise RuntimeError('ffmpeg is required; source hamr_ball_caster/scripts/env.sh')
        temporary=video.with_suffix('.partial.mp4')
        if temporary.exists(): raise ValueError(f'Unfinished output exists: {temporary}; choose a new output directory')
        command=[ffmpeg,'-nostdin','-hide_banner','-loglevel','error','-n','-f','rawvideo',
                 '-pixel_format','rgb24','-video_size',f'{args.width}x{args.width*9//16}',
                 '-framerate',str(args.fps),'-i','pipe:0','-an','-c:v','libx264','-preset','fast',
                 '-crf','18','-threads','4','-pix_fmt','yuv420p','-movflags','+faststart',str(temporary)]
        wall=time.monotonic()
        with video.with_suffix('.encoding.log').open('w') as log:
            encoder=subprocess.Popen(command,stdin=subprocess.PIPE,stderr=log)
            try:
                with video.with_suffix('.frames.csv').open('w',newline='') as stream:
                    writer=csv.writer(stream)
                    writer.writerow(['frame','video_time_s','bag_time_ns','elapsed_bag_s','h1_rad','h2_rad','center_rad','h1_state','h2_state','center_state','wheel_id','h1_device_id','h2_device_id','center_device_id'])
                    for i,frame in enumerate(frames):
                        image=draw_frame(frame)
                        encoder.stdin.write(image.tobytes())
                        nodes=frame['status']['nodes']
                        writer.writerow([i,i/args.fps,frame['stamp_ns'],(frame['stamp_ns']-start)/1e9]+[n['model_position_rad'] for n in nodes]+[n['display_state'] for n in nodes]+[wheel_id]+[n['device_id'] for n in nodes])
                        if i in (min(len(frames)-1,args.fps),len(frames)//2,len(frames)-1):
                            image.save(output/f'{video.stem}_frame_{i:05d}.png')
                        if i%(args.fps*5)==0:print(f'{bag.stem}: {i}/{len(frames)} frames; wall {time.monotonic()-wall:.1f}s',flush=True)
                encoder.stdin.close()
                if encoder.wait(timeout=120):raise RuntimeError('ffmpeg failed; inspect the encoding log')
            finally:
                if encoder.poll() is None:
                    encoder.terminate();encoder.wait(timeout=15)
        temporary.replace(video)
        summary={'bag':str(bag),'wheel_id':wheel_id,'packet_topic':topic,'video':str(video),'rate':rate,'fps':args.fps,'frames':len(frames),
                 'width':args.width,'height':args.width*9//16,'bag_start_ns':start,'bag_end_ns':end,
                 'bag_duration_s':(end-start)/1e9,'video_duration_s':len(frames)/args.fps,
                 'duration_padding_s':len(frames)/args.fps-(end-start)/1e9/rate,
                 'timing':'Frame i uses bag start + i * playback rate / fps; last valid received sample; no interpolation.',
                 'mode':'measured_motion_kinematic_replay','model_sha256':digest(model),
                 'renderer_sha256':digest(__file__),'replay_model_sha256':digest(Path(__file__).with_name('caster_replay_model.py')),
                 'video_sha256':digest(video),'packet_messages':len(events),
                 'mapping':frames[-1]['status']['mapping'],'full_visual_detail':args.full_detail,
                 'start_pose':start_pose,'ground_z_m':scene.ground_z,
                 'hidden_internal_visual_meshes':[] if args.full_detail else sorted(HIDDEN_MESHES),
                 'display_states':{role:dict(Counter(f['status']['nodes'][j]['display_state'] for f in frames)) for j,role in enumerate(['H1','H2','CENTER'])},
                 'final_rendered_status':frames[-1]['status'],'wall_render_s':time.monotonic()-wall}
        inputs=[bag] if bag.is_file() else sorted(p for p in bag.iterdir() if p.suffix in ('.mcap','.db3','.yaml'))
        summary['input_sha256']={p.name:digest(p) for p in inputs}
        if local_recording:
            summary['local_recording'] = local_recording
            summary['timing'] = local_recording['time_basis'] + ' Last valid sample at each frame; no interpolation.'
            summary['input_sha256'][provenance_path.name] = digest(provenance_path)
        if plots:
            plots.export(video)
            summary['encoder_plots']=plots.metadata
            summary['encoder_plots_sha256']=digest(Path(__file__).with_name('caster_encoder_plots.py'))
        video.with_suffix('.json').write_text(json.dumps(summary,indent=2)+'\n')
        print(video,flush=True)
    finally:
        scene.window.Finalize()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag',type=Path)
    parser.add_argument('--output-dir',type=Path,default=Path.home()/'Videos/hamr_sim/caster_encoder_replay')
    parser.add_argument('--wheel',choices=WHEELS,help='Required when the bag contains both wheels')
    parser.add_argument('--topic',help='Packet topic; inferred from --wheel or a unique bag topic')
    parser.add_argument('--rate',type=float,default=1.)
    parser.add_argument('--fps',type=int,default=30)
    parser.add_argument('--width',type=int,default=1600)
    parser.add_argument('--mapping-config',type=Path)
    parser.add_argument('--start-pose',choices=('recorded','ground-contact'),default='recorded',
                        help='ground-contact assumes the first complete pose has H1 forward and H2 back at the floor; applies measured changes thereafter')
    parser.add_argument('--plots',action='store_true',help='Show synchronized H1/H2/CENTER angle plots; also export PNG, PDF and full-sample CSV')
    parser.add_argument('--full-detail',action='store_true',help='Also render dense internal bearing and fastener visuals')
    parser.add_argument('--preview-time',type=float,help='Render one preview PNG at this elapsed bag time instead of a video')
    args=parser.parse_args()
    try:
        render(args)
    except (ValueError,RuntimeError,OSError) as exc:
        parser.exit(2,f'{exc}\n')


if __name__=='__main__':
    main()
