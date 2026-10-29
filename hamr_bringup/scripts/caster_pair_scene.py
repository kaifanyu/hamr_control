#!/usr/bin/env python3
"""Replay both casters on the full vehicle model used by Simulation Studio.

The source is the same recording Xacro selected by hamr_sim_ui through the
terrain launch. Native mesh dimensions, mounting transforms and caster joint
origins are retained. Only the recorded caster angles change; the vehicle body
and other unmeasured joints stay at their nominal pose.
"""
from __future__ import annotations

from copy import deepcopy
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from render_caster_encoder_video import (
    HIDDEN_MESHES, link_transforms, origin, package_share,
)


WHEELS = ("front_left", "front_right")
CASTER_PREFIXES = {"front_left": "left_caster_", "front_right": "right_caster_"}
MODEL_FILE = "compa_ball_caster_recording.urdf.xacro"


def model_name(wheel, standalone_name):
    """Map the standalone replay coordinates to the simulation's native names."""
    if not standalone_name.startswith("caster_"):
        raise ValueError(f"Expected a standalone caster name: {standalone_name}")
    return CASTER_PREFIXES[wheel] + standalone_name.removeprefix("caster_")


def build_robot(source_robot=None):
    """Return the Simulation Studio visual kinematics and source provenance.

    Expanding the same wrapper and camera flag as terrain_sim.launch.py keeps
    the complete chassis and native caster geometry in one authoritative model.
    No dimensions, transforms, names or carrier phases are rewritten here.
    """
    model_path = package_share("hamr_ball_caster") / "urdf" / MODEL_FILE
    if source_robot is None:
        import xacro
        source_robot = ET.fromstring(xacro.process_file(
            str(model_path), mappings={"enable_closeup_camera": "false"}).toxml())
    robot = deepcopy(source_robot)
    transforms = link_transforms(robot, {})
    centers = {wheel: transforms[model_name(wheel, "caster_fork_link")][:3, 3].tolist()
               for wheel in WHEELS}
    for element in list(robot):
        if element.tag not in ("link", "joint", "material"):
            robot.remove(element)
    for link in robot.findall("link"):
        for child in list(link):
            if child.tag != "visual":
                link.remove(child)
    provenance = {
        "chassis_model": str(model_path),
        "basis": "Unscaled full ball-caster vehicle used by hamr_sim_ui / Simulation Studio",
        "simulation_launch": "hamr_terrain/launch/terrain_sim.launch.py",
        "xacro_mappings": {"enable_closeup_camera": "false"},
        "source_model_sha256": hashlib.sha256(model_path.read_bytes()).hexdigest(),
        "expanded_visual_model_sha256": hashlib.sha256(ET.tostring(robot)).hexdigest(),
        "caster_uniform_scale": 1.0,
        "native_caster_nominal_radius_m": 0.100,
        "ball_centers_in_base_footprint_m": centers,
        "caster_joint_prefixes": dict(CASTER_PREFIXES),
        "mounting_note": "Native simulation fork mounts, mesh scales and joint origins retained unchanged.",
        "carrier_phase_rad": 0.0,
        "body_pose": "Fixed at the simulation model's neutral posture; trajectory is plotted separately",
        "unmeasured_joints": "Drive wheels, rocker/gimbal linkage and polar rollers held at zero",
        "joint_angle_scale": 1.0,
        "incomplete_mesh_fallbacks": [],
    }
    return robot, provenance


def paired_joint_angles(frames):
    """Apply each measured rotation to its native left/right simulation joint."""
    return {model_name(wheel, joint): angle for wheel in WHEELS
            for joint, angle in frames[wheel]["joints"].items()}


class Scene:
    """Full image canvas; left half contains robot view and two caster closeups."""

    def __init__(self, width, height, full_detail=False):
        import vtk
        from vtk.util.numpy_support import vtk_to_numpy
        self.vtk, self.to_numpy = vtk, vtk_to_numpy
        self.robot, self.provenance = build_robot()
        self.width, self.height = width, height
        self.window = vtk.vtkRenderWindow()
        self.window.SetOffScreenRendering(1)
        self.window.SetSize(width, height)
        self.window.SetMultiSamples(0)
        self.background = (.925, .945, .965)
        background = vtk.vtkRenderer()
        background.SetBackground(*self.background)
        self.window.AddRenderer(background)
        self.renderers = {}
        self.actors = []
        self.shell_actors = []
        self.meshes = {}
        centers = self.provenance["ball_centers_in_base_footprint_m"]
        self._renderer("robot", (0, .42, 620 / 1920, .8833), (1.25, -1.4, .95), (0., 0., .28), .53)
        for wheel, viewport in zip(WHEELS, [(0., .1167, .25, .4), (.25, .1167, .5, .4)]):
            center = np.asarray(centers[wheel])
            self._renderer(wheel, viewport, center + [.55, -.75, .35],
                           center + [0, 0, .035], .185)
        materials = {item.get("name"): item.find("color") for item in self.robot.findall("material")}
        for link in self.robot.findall("link"):
            name = link.get("name")
            wheel = next((wheel for wheel, prefix in CASTER_PREFIXES.items() if name.startswith(prefix)), None)
            for visual in link.findall("visual"):
                mesh = visual.find("geometry/mesh")
                transform = origin(visual.find("origin"))
                if mesh is not None:
                    uri = mesh.get("filename")
                    if not full_detail and Path(uri).name in HIDDEN_MESHES:
                        continue
                    package, relative = uri.removeprefix("package://").split("/", 1)
                    path = package_share(package) / relative
                    mapper = self._mesh(path)
                    transform[:3, :3] = transform[:3, :3] @ np.diag(
                        np.fromstring(mesh.get("scale", "1 1 1"), sep=" "))
                else:
                    box = visual.find("geometry/box")
                    cylinder = visual.find("geometry/cylinder")
                    if box is not None:
                        source = vtk.vtkCubeSource()
                        x, y, z = np.fromstring(box.get("size"), sep=" ")
                        source.SetXLength(x); source.SetYLength(y); source.SetZLength(z)
                    elif cylinder is not None:
                        source = vtk.vtkCylinderSource()
                        source.SetRadius(float(cylinder.get("radius")))
                        source.SetHeight(float(cylinder.get("length")))
                        source.SetResolution(96); source.CappingOn()
                        # VTK cylinders use +Y; URDF cylinders use +Z.
                        align = np.array([[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]])
                        transform[:3, :3] = transform[:3, :3] @ align
                    else:
                        raise ValueError(f"Unsupported HAMR visual geometry in {name}")
                    mapper = vtk.vtkPolyDataMapper()
                    mapper.SetInputConnection(source.GetOutputPort())
                actor = vtk.vtkActor(); actor.SetMapper(mapper)
                material = visual.find("material")
                color = material.find("color") if material is not None else None
                if color is None and material is not None:
                    color = materials.get(material.get("name"))
                rgba = np.fromstring(color.get("rgba"), sep=" ") if color is not None else [.65, .68, .72, 1.]
                actor.GetProperty().SetColor(*rgba[:3])
                actor.GetProperty().SetOpacity(1.)  # opaque shells/turret aid spatial reading
                actor.GetProperty().SetAmbient(.23); actor.GetProperty().SetDiffuse(.75)
                actor.GetProperty().SetSpecular(.3); actor.GetProperty().SetSpecularPower(40)
                self._add(actor, name, transform, wheel)
                if wheel and name.endswith(("positive_hemisphere_link", "negative_hemisphere_link")):
                    joint = "caster_" + name.removeprefix(CASTER_PREFIXES[wheel]).removesuffix("_link") + "_joint"
                    self.shell_actors.append((actor, wheel, joint))
        for wheel in WHEELS:
            for link, point in [("caster_positive_hemisphere_link", (-.0226, 0., .0974)),
                                ("caster_negative_hemisphere_link", (.0927, 0., .0376))]:
                sphere = vtk.vtkSphereSource()
                sphere.SetRadius(.0055)
                sphere.SetCenter(*point)
                sphere.SetThetaResolution(20); sphere.SetPhiResolution(12)
                mapper = vtk.vtkPolyDataMapper(); mapper.SetInputConnection(sphere.GetOutputPort())
                actor = vtk.vtkActor(); actor.SetMapper(mapper)
                actor.GetProperty().SetColor(1., 1., .96)
                self._add(actor, model_name(wheel, link), np.eye(4), wheel)
        self._floor()
        self.capture = vtk.vtkWindowToImageFilter()
        self.capture.SetInput(self.window)
        self.capture.SetInputBufferTypeToRGB(); self.capture.ReadFrontBufferOff()

    def _renderer(self, name, viewport, position, target, scale):
        vtk = self.vtk
        renderer = vtk.vtkRenderer(); renderer.UseFXAAOn()
        renderer.SetViewport(*viewport); renderer.SetBackground(*self.background)
        self.window.AddRenderer(renderer); self.renderers[name] = renderer
        camera = renderer.GetActiveCamera()
        camera.SetPosition(*position); camera.SetFocalPoint(*target); camera.SetViewUp(0, 0, 1)
        camera.ParallelProjectionOn(); camera.SetParallelScale(scale)
        camera.SetClippingRange(.001, 10.)
        for location, intensity in [((1, -2, 3), .85), ((-1, 1, 2), .65)]:
            light = vtk.vtkLight(); light.SetPosition(*location)
            light.SetFocalPoint(*target); light.SetIntensity(intensity)
            renderer.AddLight(light)

    def _mesh(self, path):
        if path not in self.meshes:
            if not path.is_file():
                raise ValueError(f"Missing HAMR CAD mesh: {path}")
            reader = self.vtk.vtkSTLReader(); reader.SetFileName(str(path)); reader.Update()
            if not reader.GetOutput().GetNumberOfPoints():
                raise ValueError(f"Empty HAMR CAD mesh: {path}")
            normals = self.vtk.vtkPolyDataNormals()
            normals.SetInputData(reader.GetOutput()); normals.SetFeatureAngle(30)
            normals.SplittingOn(); normals.ConsistencyOn(); normals.Update()
            mapper = self.vtk.vtkPolyDataMapper(); mapper.SetInputData(normals.GetOutput())
            self.meshes[path] = mapper
        return self.meshes[path]

    def _add(self, actor, link, local, wheel):
        self.actors.append((actor, link, local, wheel))
        self.renderers["robot"].AddActor(actor)
        if wheel:
            self.renderers[wheel].AddActor(actor)

    def _floor(self):
        vtk = self.vtk
        plane = vtk.vtkPlaneSource()
        plane.SetOrigin(-.65, -.6, -.001); plane.SetPoint1(.65, -.6, -.001)
        plane.SetPoint2(-.65, .6, -.001)
        mapper = vtk.vtkPolyDataMapper(); mapper.SetInputConnection(plane.GetOutputPort())
        actor = vtk.vtkActor(); actor.SetMapper(mapper)
        actor.GetProperty().SetColor(.81, .84, .87)
        actor.GetProperty().LightingOff()
        for renderer in self.renderers.values():
            renderer.AddActor(actor)

    def render(self, frames):
        from PIL import Image
        transforms = link_transforms(self.robot, paired_joint_angles(frames))
        # Match role colors even when an explicit mapping swaps H1 and H2's
        # physical CAD sides. Phase markers are added separately and stay white.
        for actor, wheel, joint in self.shell_actors:
            mapping = frames[wheel]["status"].get("mapping", {})
            h1 = mapping.get("h1_joint", "caster_positive_hemisphere_joint")
            actor.GetProperty().SetColor(*((.16, .46, .85) if joint == h1 else (.92, .46, .12)))
        for actor, link, local, wheel in self.actors:
            visible = wheel is None or frames[wheel]["status"]["complete_pose_available"] or link.endswith("fork_link")
            actor.SetVisibility(visible)
            pose = transforms[link] @ local
            matrix = self.vtk.vtkMatrix4x4()
            for i in range(4):
                for j in range(4):
                    matrix.SetElement(i, j, pose[i, j])
            actor.SetUserMatrix(matrix)
        self.window.Render(); self.capture.Modified(); self.capture.Update()
        pixels = self.to_numpy(self.capture.GetOutput().GetPointData().GetScalars())
        return Image.fromarray(np.flipud(pixels.reshape(self.height, self.width, 3)).copy())

    def close(self):
        self.window.Finalize()
