"""The viewing layer must not change the terrain's physical experiment."""
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from hamr_sim_ui.world import prepare_world
from hamr_sim_ui.markers import route_request


class WorldSnapshotTest(unittest.TestCase):
    def test_camera_off_removes_existing_camera_and_renderer(self):
        source = Path(__file__).resolve().parents[2] / "hamr_swivel_caster/worlds/swivel_caster_test.sdf"
        with tempfile.TemporaryDirectory() as folder:
            destination = Path(folder) / "world.sdf"
            prepare_world(source, destination, cameras=False, log_path=Path(folder) / "replay")
            before = ET.parse(source).getroot().find("world")
            after = ET.parse(destination).getroot().find("world")
            self.assertEqual(after.findall(".//sensor"), [])
            self.assertIsNone(after.find("plugin[@name='gz::sim::systems::Sensors']"))
            self.assertIsNotNone(after.find("plugin[@name='gz::sim::systems::LogRecord']"))
            for element in ("physics", "model[@name='ground_plane']",
                            "plugin[@name='gz::sim::systems::Physics']",
                            "plugin[@name='gz::sim::systems::Contact']"):
                self.assertEqual(ET.tostring(before.find(element)), ET.tostring(after.find(element)))

    def test_camera_off_preserves_other_sensors(self):
        with tempfile.TemporaryDirectory() as folder:
            source, destination = Path(folder) / "source.sdf", Path(folder) / "world.sdf"
            source.write_text('<sdf version="1.9"><world name="test">'
                              '<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"/>'
                              '<model name="sensors"><link name="link">'
                              '<sensor name="camera" type="camera"/>'
                              '<sensor name="scan" type="gpu_lidar"/>'
                              '<sensor name="contact" type="contact"/>'
                              '</link></model></world></sdf>')
            prepare_world(source, destination, cameras=False)
            world = ET.parse(destination).getroot().find("world")
            self.assertIsNone(world.find(".//sensor[@type='camera']"))
            self.assertIsNotNone(world.find(".//sensor[@type='gpu_lidar']"))
            self.assertIsNotNone(world.find(".//sensor[@type='contact']"))
            self.assertIsNotNone(world.find("plugin[@name='gz::sim::systems::Sensors']"))

    def test_preserves_physics_and_terrain(self):
        source = Path(__file__).resolve().parents[2] / "maps/worlds/road_bumps.sdf"
        with tempfile.TemporaryDirectory() as folder:
            destination = Path(folder) / "world.sdf"
            name = prepare_world(source, destination, log_path=Path(folder) / "replay")
            before = ET.parse(source).getroot().find("world")
            after = ET.parse(destination).getroot().find("world")
            self.assertEqual(name, before.get("name"))
            self.assertEqual(ET.tostring(before.find("physics")), ET.tostring(after.find("physics")))
            for original in before.findall("model"):
                copied = after.find(f"model[@name='{original.get('name')}']")
                self.assertEqual(ET.tostring(original), ET.tostring(copied))
            self.assertEqual(len(after.findall("plugin[@name='gz::sim::systems::Sensors']")), 1)
            camera = after.find("model[@name='hamr_ui_overview_camera']")
            self.assertIsNotNone(camera.find("link/sensor/camera"))
            self.assertEqual(camera.findall(".//collision"), [])

    def test_relocated_relative_mesh_and_existing_log(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            source = root / "source.sdf"
            source.write_text('<sdf version="1.9"><world name="test"><model name="terrain">'
                              '<link name="surface"><visual name="ground"><geometry><mesh>'
                              '<uri>meshes/map.dae</uri></mesh></geometry></visual></link></model>'
                              '</world></sdf>')
            destination = root / "run/world.sdf"
            prepare_world(source, destination)
            self.assertEqual(ET.parse(destination).find(".//uri").text, str(root / "meshes/map.dae"))
            replay = root / "replay"
            replay.mkdir()
            (replay / "state.tlog").write_text("existing recorded simulation")
            with self.assertRaises(ValueError):
                prepare_world(source, destination, log_path=replay)
            with self.assertRaises(ValueError):
                prepare_world(source, source)

    def test_route_markers_reject_nonfinite_inputs(self):
        with self.assertRaises(ValueError):
            route_request([(float("nan"), 0)])
        self.assertIn('ns: "hamr_ui_route" action: DELETE_ALL', route_request([]))


if __name__ == "__main__":
    unittest.main()
