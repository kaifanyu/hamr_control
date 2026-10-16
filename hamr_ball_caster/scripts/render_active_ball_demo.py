#!/usr/bin/env python3
"""Render the deterministic four-active-ball concept simulation to H.264 MP4.

This is an ideal rolling-sphere kinematic visualization, not contact dynamics.
The native Gazebo preview uses the same trajectory (active_ball_gazebo.py).
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import shutil
import subprocess
import time

import numpy as np
from PIL import Image, ImageDraw, ImageFont
import vtk
from vtk.util.numpy_support import vtk_to_numpy

from active_ball_trajectory import BALL_OFFSETS, BALL_RADIUS, TARGETS, DemoTrajectory


BG = "#101c2c"
PANEL = "#17263a"
WHITE = "#edf5fc"
MUTED = "#9daec4"
CYAN = "#45e0d2"
AMBER = "#ffbf62"
BLUE = "#526880"


def rgb(color):
    return tuple(int(color[i:i+2], 16) / 255 for i in (1, 3, 5))


def actor(source, color, metallic=False):
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(source.GetOutputPort())
    item = vtk.vtkActor()
    item.SetMapper(mapper)
    prop = item.GetProperty()
    prop.SetColor(*rgb(color))
    prop.SetAmbient(.28)
    prop.SetDiffuse(.72)
    prop.SetSpecular(.45 if metallic else .22)
    prop.SetSpecularPower(40)
    return item


def box(center, size, color):
    source = vtk.vtkCubeSource()
    source.SetCenter(*center)
    source.SetXLength(size[0])
    source.SetYLength(size[1])
    source.SetZLength(size[2])
    return actor(source, color, True)


def tube(points, radius, color, sides=10):
    pts = vtk.vtkPoints()
    line = vtk.vtkPolyLine()
    line.GetPointIds().SetNumberOfIds(len(points))
    for i, point in enumerate(points):
        pts.InsertNextPoint(*point)
        line.GetPointIds().SetId(i, i)
    cells = vtk.vtkCellArray()
    cells.InsertNextCell(line)
    data = vtk.vtkPolyData()
    data.SetPoints(pts)
    data.SetLines(cells)
    source = vtk.vtkTubeFilter()
    source.SetInputData(data)
    source.SetRadius(radius)
    source.SetNumberOfSides(sides)
    source.CappingOn()
    return actor(source, color)


def ring(x, y, radius, color, z=.014, width=.008):
    return tube([(x + radius * math.cos(t), y + radius * math.sin(t), z)
                 for t in np.linspace(0, 2 * math.pi, 65)], width, color)


def arrow(start, end, color, radius=.009):
    start, end = np.asarray(start), np.asarray(end)
    delta = end - start
    normal = np.array([-delta[1], delta[0], 0.])
    normal /= max(np.linalg.norm(normal), 1e-9)
    head = end - .10 * delta / max(np.linalg.norm(delta), 1e-9)
    return tube([start, end, head + .045 * normal, end, head - .045 * normal], radius, color)


class Scene:
    def __init__(self, width=1600, height=900):
        self.width, self.height = width, height
        self.window = vtk.vtkRenderWindow()
        self.window.SetOffScreenRendering(1)
        self.window.SetSize(width, height)
        self.window.SetMultiSamples(4)
        self.background = vtk.vtkRenderer()
        self.background.SetBackground(*rgb(BG))
        self.window.AddRenderer(self.background)
        self.main = vtk.vtkRenderer()
        self.main.SetViewport(0, .085, .705, .88)
        self.main.SetBackground(*rgb(BG))
        self.detail = vtk.vtkRenderer()
        self.detail.SetViewport(.72, .51, .985, .86)
        self.detail.SetBackground(*rgb(PANEL))
        self.window.AddRenderer(self.main)
        self.window.AddRenderer(self.detail)
        self.body = []
        self.balls = []
        self.active_path = None
        self.velocity = None
        self.last_target = None
        self.make_floor()
        self.make_robot()
        camera = self.main.GetActiveCamera()
        camera.SetPosition(4.8, -6.9, 7.4)
        camera.SetFocalPoint(0, 0, .02)
        camera.SetViewUp(0, 0, 1)
        camera.ParallelProjectionOn()
        camera.SetParallelScale(2.13)
        close = self.detail.GetActiveCamera()
        close.ParallelProjectionOn()
        close.SetParallelScale(.53)
        close.SetViewUp(0, 0, 1)
        for renderer in (self.main, self.detail):
            light = vtk.vtkLight()
            light.SetLightTypeToSceneLight()
            light.SetPosition(2, -3, 7)
            light.SetFocalPoint(0, 0, 0)
            light.SetIntensity(.85)
            renderer.AddLight(light)
            fill = vtk.vtkLight()
            fill.SetLightTypeToCameraLight()
            fill.SetIntensity(.65)
            renderer.AddLight(fill)
        self.capture = vtk.vtkWindowToImageFilter()
        self.capture.SetInput(self.window)
        self.capture.SetInputBufferTypeToRGB()
        self.capture.ReadFrontBufferOff()

    def make_floor(self):
        self.main.AddActor(box((0, 0, -.055), (3.7, 3.7, .10), "#26384d"))
        for coordinate in np.arange(-1.8, 1.81, .3):
            self.main.AddActor(tube([(coordinate, -1.8, .001), (coordinate, 1.8, .001)], .0025, "#34485e", 6))
            self.main.AddActor(tube([(-1.8, coordinate, .001), (1.8, coordinate, .001)], .0025, "#34485e", 6))
        self.main.AddActor(tube([(-1.2, -1.2, .008), (1.2, -1.2, .008), (1.2, 1.2, .008),
                                 (-1.2, 1.2, .008), (-1.2, -1.2, .008)], .012, "#b8cddd"))
        for target in TARGETS:
            x, y = target['xy']
            # Eight center-to-target spokes remain visible below the active leg.
            self.main.AddActor(tube([(0, 0, .006), (x, y, .006)], .004, "#526c80", 6))
            self.main.AddActor(ring(x, y, .077, "#9db3c8", width=.006))
        self.main.AddActor(ring(0, 0, .115, AMBER, width=.009))
        self.main.AddActor(arrow((-1.63, -1.58, .016), (-1.20, -1.58, .016), AMBER, .007))
        self.main.AddActor(arrow((-1.63, -1.58, .016), (-1.63, -1.15, .016), CYAN, .007))
        self.target_rings = []
        for target in TARGETS:
            x, y = target['xy']
            marker = ring(x, y, .093, CYAN, width=.010)
            marker.SetVisibility(False)
            self.main.AddActor(marker)
            self.target_rings.append(marker)

    def add_body(self, item):
        self.body.append(item)
        self.main.AddActor(item)
        self.detail.AddActor(item)

    def make_robot(self):
        self.add_body(box((0, 0, .28), (.58, .46, .12), "#b9c7d3"))
        self.add_body(box((0, 0, .345), (.54, .42, .022), "#41576d"))
        self.add_body(box((-.02, 0, .373), (.42, .34, .035), "#263c53"))
        self.add_body(box((-.05, 0, .397), (.26, .22, .018), "#527085"))
        # Two side rails and front lighting emphasize the fixed body heading.
        for side in (-1, 1):
            self.add_body(box((0, side * .225, .323), (.49, .012, .025), CYAN))
            self.add_body(box((.294, side * .142, .29), (.014, .09, .018), AMBER))
        self.add_body(arrow((-.1, 0, .417), (.225, 0, .417), AMBER, .012))
        for i, (x, y) in enumerate(BALL_OFFSETS):
            self.add_body(box((x * .91, y * .80, .23), (.13, .16, .045), "#667f92"))
            # Stylized orthogonal actuator housings; no transmission design implied.
            self.add_body(box((x, y, .228), (.13, .13, .050), "#a7bacb"))
            self.add_body(box((x + .065, y, .22), (.085, .065, .058), AMBER))
            self.add_body(box((x, y + .066, .22), (.063, .086, .058), "#45c5c1"))
            sphere = vtk.vtkSphereSource()
            sphere.SetRadius(BALL_RADIUS)
            sphere.SetThetaResolution(48)
            sphere.SetPhiResolution(32)
            sphere.Update()
            data = sphere.GetOutput()
            colors = vtk.vtkUnsignedCharArray()
            colors.SetNumberOfComponents(3)
            colors.SetName("rolling_stripes")
            for cell_id in range(data.GetNumberOfCells()):
                cell = data.GetCell(cell_id)
                p = np.mean([data.GetPoint(cell.GetPointId(j)) for j in range(cell.GetNumberOfPoints())], axis=0)
                longitude = math.atan2(p[1], p[0])
                stripe = int((longitude + math.pi) / (math.pi / 4)) % 2 == 0
                color = "#37d5d0" if stripe else "#d5e4eb"
                if abs(p[2]) > BALL_RADIUS * .88:
                    color = "#3b556d"
                colors.InsertNextTuple3(*(int(c * 255) for c in rgb(color)))
            data.GetCellData().SetScalars(colors)
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputData(data)
            ball = vtk.vtkActor()
            ball.SetMapper(mapper)
            ball.GetProperty().SetAmbient(.25)
            ball.GetProperty().SetSpecular(.6)
            ball.GetProperty().SetSpecularPower(48)
            self.balls.append(ball)
            self.main.AddActor(ball)
            self.detail.AddActor(ball)
        # Soft floor contact shadows are graphic cues, not a contact solver.
        self.shadows = []
        for x, y in BALL_OFFSETS:
            source = vtk.vtkDiskSource()
            source.SetInnerRadius(0)
            source.SetOuterRadius(.108)
            source.SetCircumferentialResolution(40)
            shadow = actor(source, "#172738")
            shadow.SetPosition(x, y, .004)
            self.shadows.append(shadow)
            self.main.AddActor(shadow)

    def render(self, state):
        x, y = state['x'], state['y']
        for item in self.body:
            item.SetPosition(x, y, 0)
        for i, ball in enumerate(self.balls):
            transform = vtk.vtkMatrix4x4()
            for row in range(3):
                for col in range(3):
                    transform.SetElement(row, col, state['ball_rotations'][i][row, col])
            transform.SetElement(0, 3, x + BALL_OFFSETS[i][0])
            transform.SetElement(1, 3, y + BALL_OFFSETS[i][1])
            transform.SetElement(2, 3, BALL_RADIUS)
            ball.SetUserMatrix(transform)
            self.shadows[i].SetPosition(x + BALL_OFFSETS[i][0], y + BALL_OFFSETS[i][1], .004)
        index = max(0, state['target_index']) if state['phase'] != 'finish_hold' else 7
        if index != self.last_target:
            if self.active_path:
                self.main.RemoveActor(self.active_path)
            tx, ty = TARGETS[index]['xy']
            self.active_path = tube([(0, 0, .014), (tx, ty, .014)], .012, CYAN)
            self.main.AddActor(self.active_path)
            self.last_target = index
        self.active_path.SetVisibility(state['phase'] not in ('start_hold', 'finish_hold'))
        for i, marker in enumerate(self.target_rings):
            marker.SetVisibility(i < state['completed_targets'] or (i == index and state['phase'] != 'start_hold'))
        if self.velocity:
            self.main.RemoveActor(self.velocity)
        speed = math.hypot(state['vx'], state['vy'])
        if speed > .005:
            v = np.array([state['vx'], state['vy'], 0]) / speed
            start = np.array([x, y, .03]) + v * .46
            end = start + v * (.2 + speed * .50)
            self.velocity = arrow(start, end, CYAN, .013)
            self.main.AddActor(self.velocity)
        else:
            self.velocity = None
        close = self.detail.GetActiveCamera()
        close.SetPosition(x + .95, y - 1.35, .96)
        close.SetFocalPoint(x, y, .20)
        self.main.ResetCameraClippingRange()
        self.detail.ResetCameraClippingRange()
        self.window.Render()
        self.capture.Modified()
        self.capture.Update()
        pixels = vtk_to_numpy(self.capture.GetOutput().GetPointData().GetScalars())
        return Image.fromarray(np.flipud(pixels.reshape(self.height, self.width, 3)).copy())

    def project(self, point):
        self.main.SetWorldPoint(*point, 1)
        self.main.WorldToDisplay()
        x, y, _ = self.main.GetDisplayPoint()
        return x, self.height - y


class Overlay:
    def __init__(self, width, height, duration):
        self.width, self.height, self.duration = width, height, duration
        self.scale = width / 1600
        regular = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
        bold = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
        mono = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"
        self.fonts = {key: ImageFont.truetype(path, round(size * self.scale))
                      for key, path, size in [('title', bold, 36), ('sub', regular, 17),
                                              ('label', bold, 15), ('small', regular, 14),
                                              ('value', mono, 24), ('tiny', regular, 12)]}

    def draw(self, image, state, scene):
        d = ImageDraw.Draw(image)
        s = self.scale

        def rect(coords, fill, outline=None, width=1, radius=0):
            coords = tuple(round(n * s) for n in coords)
            if radius:
                d.rounded_rectangle(coords, round(radius*s), fill, outline, width=width)
            else:
                d.rectangle(coords, fill, outline, width=width)

        def text(x, y, value, font='sub', fill=WHITE, anchor=None):
            d.text((x*s, y*s), value, font=self.fonts[font], fill=fill, anchor=anchor)

        rect((0, 0, 1600, 112), BG)
        text(38, 22, "Four active ball casters", 'title')
        text(40, 73, "HOLONOMIC MOTION     /     Center to corners + edge midpoints", 'sub', MUTED)
        rect((1170, 27, 1563, 71), PANEL, radius=8)
        text(1366, 49, "KINEMATIC CONCEPT  •  FIXED HEADING", 'label', CYAN, 'mm')
        text(42, 122, "BOX VIEW", 'label', MUTED)
        text(42, 147, "2.4 × 2.4 m   /   +X forward", 'small', MUTED)
        # The close-up has its own tracking camera but the chassis does not yaw.
        text(1162, 120, "FOUR POWERED SPHERES", 'label', WHITE)
        text(1162, 145, "Rolling direction changes; body heading holds.", 'tiny', MUTED)
        rect((1152, 439, 1575, 443), BG)
        text(1164, 418, "Cyan = motion     Amber = body forward", 'small', MUTED)

        # Floor labels project from world coordinates so they stay on the box.
        for i, target in enumerate(TARGETS):
            tx, ty = target['xy']
            px, py = scene.project((tx * 1.19, ty * 1.19, .018))
            color = CYAN if i < state['completed_targets'] or i == state['target_index'] else MUTED
            d.text((px, py), f"{i+1}  {target['label']}", font=self.fonts['label'], fill=color, anchor='mm')
        px, py = scene.project((-1.1, -1.58, .02))
        d.text((px, py), '+X', font=self.fonts['tiny'], fill=AMBER, anchor='mm')
        px, py = scene.project((-1.63, -1.03, .02))
        d.text((px, py), '+Y', font=self.fonts['tiny'], fill=CYAN, anchor='mm')

        rect((1152, 460, 1575, 799), PANEL, radius=12)
        text(1174, 478, 'TOP VIEW / TARGETS', 'label', MUTED)
        cx, cy, span = 1292, 633, 97

        def map_point(x, y):
            return ((cx + x / 1.2 * span)*s, (cy - y / 1.2 * span)*s)

        d.rectangle([map_point(-1.2, 1.2), map_point(1.2, -1.2)], outline=BLUE, width=2)
        for i, target in enumerate(TARGETS):
            end = map_point(*target['xy'])
            color = CYAN if i == state['target_index'] else BLUE
            d.line([map_point(0, 0), end], fill=color, width=2)
            r = 5*s
            d.ellipse((end[0]-r, end[1]-r, end[0]+r, end[1]+r), fill=CYAN if i < state['completed_targets'] else PANEL, outline=color, width=2)
            off = np.asarray(target['xy']) * 1.20
            d.text(map_point(*off), target['label'], font=self.fonts['tiny'], fill=MUTED, anchor='mm')
        rx, ry = map_point(state['x'], state['y'])
        for ox, oy in BALL_OFFSETS:
            bx, by = map_point(state['x'] + ox, state['y'] + oy)
            d.ellipse((bx-7*s, by-7*s, bx+7*s, by+7*s), fill=CYAN)
        d.rounded_rectangle((rx-23*s, ry-18*s, rx+23*s, ry+18*s), radius=3*s, fill=WHITE)
        d.polygon([(rx+19*s, ry), (rx+4*s, ry-6*s), (rx+4*s, ry+6*s)], fill=AMBER)
        speed = math.hypot(state['vx'], state['vy'])
        text(1423, 528, 'HEADING', 'tiny', MUTED)
        text(1423, 549, '0.0°', 'value', AMBER)
        text(1423, 586, 'SPEED', 'tiny', MUTED)
        text(1423, 607, f'{speed:.2f}', 'value', WHITE)
        text(1423, 639, 'm/s', 'tiny', MUTED)
        text(1423, 678, 'VISITED', 'tiny', MUTED)
        text(1423, 699, f"{state['completed_targets']} / 8", 'value', CYAN)
        text(1176, 767, f"x {state['x']:+.2f} m    y {state['y']:+.2f} m", 'small', MUTED)

        phase = state['phase']
        index = max(0, state['target_index']) if state['phase'] != 'finish_hold' else 7
        title = f"{index+1:02d} / 08   {TARGETS[index]['name']}"
        detail = {
            'start_hold': 'Start at center  •  heading locked to +X',
            'outbound': 'Center → target  •  direct translation',
            'target_hold': 'Target reached  •  pause',
            'return': 'Target → center  •  same body heading',
            'center_hold': 'Back at center  •  next direction',
            'finish_hold': 'All 8 targets visited  •  returned to center',
        }[phase]
        if phase == 'finish_hold':
            title = '08 / 08   Demonstration complete'
        rect((37, 719, 648, 800), BG, radius=8)
        text(52, 729, title, 'label', CYAN)
        text(52, 760, detail, 'sub', WHITE)
        rect((0, 819, 1600, 900), BG)
        text(40, 838, 'Ideal active-ball kinematics • hard-coded trajectory • conceptual actuator housings', 'small', MUTED)
        text(1558, 841, f"{state['time']:05.1f} / {self.duration:04.1f} s", 'small', WHITE, 'ra')
        for i in range(8):
            left = 40 + i * 191
            rect((left, 877, left+179, 881), BLUE)
            if i < state['target_index'] or phase == 'finish_hold':
                rect((left, 877, left+179, 881), CYAN)
            elif i == state['target_index']:
                rect((left, 877, left+179, 881), AMBER)
        return image


def find_ffmpeg():
    found = shutil.which('ffmpeg')
    fallback = Path.home() / '.cache/hamr-gazebo-root/usr/bin/ffmpeg'
    if found:
        return found
    if fallback.exists():
        return str(fallback)
    raise RuntimeError('ffmpeg is required; install it or source hamr_ball_caster/scripts/env.sh')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, default=Path.home() / 'Videos/hamr_sim/active_ball_demo')
    parser.add_argument('--fps', type=int, default=30)
    parser.add_argument('--width', type=int, default=1600)
    parser.add_argument('--preview-time', type=float, help='Render just one PNG at this simulation time')
    args = parser.parse_args()
    if args.fps < 1 or args.fps > 120 or args.width < 800 or args.width % 32:
        parser.error('fps must be 1..120; width must be >=800 and divisible by 32 (16:9 with even dimensions)')
    output = args.output_dir.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    trajectory = DemoTrajectory()
    height = args.width * 9 // 16
    scene = Scene(args.width, height)
    overlay = Overlay(args.width, height, trajectory.duration)
    if args.preview_time is not None:
        state = trajectory.sample(args.preview_time)
        path = output / f'preview_{args.preview_time:05.1f}.png'
        overlay.draw(scene.render(state), state, scene).save(path)
        print(path)
        return
    video = output / 'four_active_ball_casters.mp4'
    if video.exists():
        parser.error(f'Refusing to overwrite {video}; choose a new --output-dir')
    frame_count = math.ceil(trajectory.duration * args.fps) + 1
    log = (output / 'encoding.log').open('w')
    encoder = subprocess.Popen([
        find_ffmpeg(), '-nostdin', '-hide_banner', '-loglevel', 'warning',
        '-f', 'rawvideo', '-pixel_format', 'rgb24', '-video_size', f'{args.width}x{height}',
        '-framerate', str(args.fps), '-i', 'pipe:0', '-an', '-c:v', 'libx264',
        '-preset', 'fast', '-crf', '18', '-threads', '4', '-pix_fmt', 'yuv420p',
        '-movflags', '+faststart', str(video),
    ], stdin=subprocess.PIPE, stderr=log)
    started = time.monotonic()
    preview_frames = {0, round(trajectory.duration * args.fps * .05), round(trajectory.duration * args.fps * .38), frame_count-1}
    csv_path = output / 'trajectory.csv'
    try:
        with csv_path.open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['time_s', 'x_m', 'y_m', 'yaw_rad', 'vx_m_s', 'vy_m_s', 'yaw_rate_rad_s',
                             'phase', 'target', 'visited', 'ball_omega_x_rad_s', 'ball_omega_y_rad_s'])
            for frame in range(frame_count):
                state = trajectory.sample(min(frame / args.fps, trajectory.duration))
                image = overlay.draw(scene.render(state), state, scene)
                encoder.stdin.write(image.tobytes())
                writer.writerow([state[k] for k in ('time', 'x', 'y', 'yaw', 'vx', 'vy', 'yaw_rate', 'phase', 'target_name', 'completed_targets')]
                                + list(state['ball_omega_world'][0][:2]))
                if frame in preview_frames:
                    image.save(output / f'frame_{frame:05d}.png')
                if frame % (args.fps * 5) == 0:
                    print(f'Rendering {state["time"]:.1f}/{trajectory.duration:.1f}s ({frame}/{frame_count} frames); wall {time.monotonic()-started:.0f}s', flush=True)
        encoder.stdin.close()
        result = encoder.wait(timeout=120)
        if result:
            raise RuntimeError(f'ffmpeg exited with {result}; see {output / "encoding.log"}')
    finally:
        if encoder.poll() is None:
            encoder.terminate()
            encoder.wait(timeout=15)
        log.close()
        scene.window.Finalize()
    summary = {
        'video': str(video), 'simulation': 'ideal no-slip active spherical wheels; prescribed pose, no contact or actuator dynamics',
        'renderer': 'VTK deterministic fixed-step replay; same trajectory as native Gazebo preview',
        'width': args.width, 'height': height, 'fps': args.fps, 'frames': frame_count,
        'simulation_duration_s': trajectory.duration, 'video_duration_s': frame_count / args.fps,
        'wall_render_s': time.monotonic() - started, 'box_size_m': [2.4, 2.4],
        'ball_radius_m': BALL_RADIUS, 'ball_offsets_m': np.asarray(BALL_OFFSETS).tolist(),
        'heading_rad': 0, 'targets': TARGETS,
        'route': 'center -> each target -> center, in listed target order',
        'completed_targets': int(trajectory.sample(trajectory.duration)['completed_targets']),
        'final_xy_m': [trajectory.sample(trajectory.duration)[k] for k in ('x', 'y')],
        'trajectory_csv': str(csv_path), 'schedule': trajectory.summary(),
    }
    (output / 'run_summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(video, flush=True)


if __name__ == '__main__':
    main()
