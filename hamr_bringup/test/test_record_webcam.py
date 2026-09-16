"""Exercise recording lifecycle and timestamp preservation without ROS hardware."""

import csv
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'record_webcam.py'
SPEC = importlib.util.spec_from_file_location('record_webcam', SCRIPT)
RECORDER = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = RECORDER
SPEC.loader.exec_module(RECORDER)


class FakeImage:
    shape = (480, 640, 3)


class FakeWriter:
    def __init__(self, opened=True, write_error=None):
        self.opened = opened
        self.write_error = write_error
        self.frames = []
        self.releases = 0

    def isOpened(self):
        return self.opened

    def write(self, image):
        if self.write_error:
            raise self.write_error
        self.frames.append(image)

    def release(self):
        self.releases += 1
        self.opened = False


class RecordingTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.output = Path(self.temporary.name)
        self.writer = FakeWriter()
        self.published = []

    def session(self, **kwargs):
        session = RECORDER.RecordingSession(
            self.output, lambda *_: self.writer,
            on_frame=lambda index, sample: self.published.append((index, sample.image_stamp_ns)),
            **kwargs,
        )
        self.addCleanup(session.close)
        return session

    @staticmethod
    def sample(t, stamp=None):
        return RECORDER.FrameSample(
            FakeImage(), t if stamp is None else stamp, t + 10, t + 20, t, 'camera_optical')

    def rows(self):
        with (self.output / 'frame_timestamps.csv').open(newline='') as source:
            return list(csv.DictReader(source))

    def metadata(self):
        return json.loads((self.output / 'video_metadata.json').read_text(encoding='utf-8'))

    def test_motion_gates_clip_and_zero_command_does_not_pause(self):
        session = self.session()
        session.receive(self.sample(1_000_000_000))
        self.assertEqual(session.state, 'armed')
        self.assertEqual(self.writer.frames, [])
        for value in (0.0, 0.01, -0.01, float('nan'), float('inf')):
            self.assertFalse(session.observe_motion(value, 'left', 1, 2, 1_100_000_000))
        self.assertTrue(session.observe_motion(-0.02, 'left', 1, 2, 1_100_000_000))
        self.assertEqual(session.state, 'recording')
        self.assertFalse(session.observe_motion(0.0, 'left', 3, 4, 1_200_000_000))
        session.receive(self.sample(1_200_000_000))
        session.close()
        self.assertEqual(len(self.writer.frames), 2)
        self.assertEqual(self.metadata()['motion_trigger']['command_rad_s'], -0.02)

    def test_preroll_is_bounded_by_time_and_command_does_not_encode_stale_frames(self):
        session = self.session()
        for index in range(100):
            session.receive(self.sample(index * 10_000_000))
        self.assertLessEqual(len(session.preroll), session.preroll.maxlen)
        self.assertTrue(session.observe_motion(1.0, 'left', 1, 2, 2_000_000_000))
        self.assertEqual(session.frame_count, 0)
        session.receive(self.sample(2_010_000_000))
        self.assertEqual(session.frame_count, 1)

    def test_preroll_preserves_chronology_with_live_frames(self):
        session = self.session()
        for t in (100_000_000, 600_000_000, 900_000_000):
            session.receive(self.sample(t))
        session.observe_motion(1.0, 'left', 1, 2, 1_000_000_000)
        session.receive(self.sample(1_100_000_000))
        session.close()
        self.assertEqual(self.published, [
            (0, 600_000_000), (1, 900_000_000), (2, 1_100_000_000)])
        self.assertEqual(len(self.rows()), 3)

    def test_irregular_source_timestamps_are_not_replaced_by_video_time(self):
        session = self.session(start_on_motion=False, video_fps=25.0)
        for t, stamp in ((1, 8_000_000_000), (2, 8_100_000_000), (3, 8_050_000_000)):
            session.receive(self.sample(t, stamp=stamp))
        session.close()
        rows = self.rows()
        self.assertEqual([int(row['image_stamp_ns']) for row in rows],
                         [8_000_000_000, 8_100_000_000, 8_050_000_000])
        self.assertEqual([float(row['video_time_sec']) for row in rows], [0, 0.04, 0.08])
        self.assertEqual([int(row['receive_monotonic_ns']) for row in rows], [1, 2, 3])
        self.assertEqual([int(row['receive_ros_ns']) for row in rows], [11, 12, 13])
        self.assertEqual([int(row['receive_unix_ns']) for row in rows], [21, 22, 23])
        self.assertEqual(self.metadata()['nonincreasing_image_timestamps'], 1)

    def test_encoder_initialization_failure_produces_no_timestamp_row(self):
        self.writer.opened = False
        session = self.session(start_on_motion=False)
        with self.assertRaisesRegex(RuntimeError, 'Cannot open') as error:
            session.receive(self.sample(1))
        session.close(error=error.exception)
        self.assertEqual(self.rows(), [])
        self.assertEqual(self.writer.releases, 1)
        self.assertEqual(self.metadata()['state'], 'failed')

    def test_write_failure_keeps_only_successful_frame_rows(self):
        session = self.session(start_on_motion=False)
        session.receive(self.sample(1))
        self.writer.write_error = OSError('disk full')
        with self.assertRaisesRegex(OSError, 'disk full') as error:
            session.receive(self.sample(2))
        session.close(error=error.exception)
        self.assertEqual(len(self.rows()), 1)
        self.assertEqual(self.published, [(0, 1)])
        self.assertEqual(self.metadata()['error'], 'disk full')

    def test_resolution_change_is_fatal_and_does_not_add_timestamp_row(self):
        session = self.session(start_on_motion=False)
        session.receive(self.sample(1))
        different = self.sample(2)
        different.image.shape = (720, 1280, 3)
        with self.assertRaisesRegex(ValueError, 'resolution changed') as error:
            session.receive(different)
        session.close(error=error.exception)
        self.assertEqual(len(self.rows()), 1)

    def test_finalization_failure_still_closes_files_and_records_failure(self):
        def invalid_video(_path, _frames):
            raise RuntimeError('finalized container frame mismatch')

        session = self.session(start_on_motion=False, verify_video=invalid_video)
        session.receive(self.sample(1))
        with self.assertRaisesRegex(RuntimeError, 'frame mismatch'):
            session.close(stop_reason='SIGINT')
        self.assertTrue(session.csv_file.closed)
        self.assertEqual(self.metadata()['state'], 'failed')
        self.assertEqual(self.metadata()['stop_reason'], 'SIGINT')
        session.close()
        self.assertEqual(self.writer.releases, 1)

    def test_close_without_motion_saves_metadata_without_opening_encoder(self):
        session = self.session()
        session.receive(self.sample(1))
        session.close(stop_reason='SIGTERM')
        session.close()
        self.assertEqual(self.writer.releases, 0)
        self.assertEqual(self.rows(), [])
        self.assertEqual(self.metadata()['state'], 'no_frames')
        self.assertIsNone(self.metadata()['video_file'])
        self.assertTrue(session.csv_file.closed)

    def test_existing_recording_is_not_overwritten(self):
        original = self.output / 'video.mp4'
        original.write_bytes(b'existing footage')
        with self.assertRaises(FileExistsError):
            self.session()
        self.assertEqual(original.read_bytes(), b'existing footage')

    def test_watchdogs_use_monotonic_time_before_and_after_first_image(self):
        session = self.session()
        start = session.created_monotonic_ns
        with self.assertRaisesRegex(RuntimeError, 'startup timeout'):
            session.check_health(start + 16_000_000_000)
        session.receive(self.sample(start + 17_000_000_000, stamp=0))
        with self.assertRaisesRegex(RuntimeError, 'stream stopped'):
            session.check_health(start + 23_000_000_000)

    def test_opencv_clip_contains_exactly_the_timestamped_frames(self):
        try:
            import cv2
            import numpy as np
        except ImportError:
            self.skipTest('Optional OpenCV integration test requires cv2 and numpy')

        session = RECORDER.RecordingSession(
            self.output,
            lambda path, codec, fps, size: cv2.VideoWriter(
                path, cv2.VideoWriter_fourcc(*codec), fps, size),
            start_on_motion=False,
        )
        self.addCleanup(session.close)
        for index in range(5):
            sample = self.sample(index * 50_000_000)
            sample.image = np.full((48, 64, 3), index * 40, dtype=np.uint8)
            session.receive(sample)
        session.close()
        capture = cv2.VideoCapture(str(session.video_path))
        try:
            self.assertTrue(capture.isOpened())
            self.assertEqual(int(capture.get(cv2.CAP_PROP_FRAME_COUNT)), 5)
            decoded = 0
            while True:
                success, image = capture.read()
                if not success:
                    break
                self.assertEqual(image.shape, (48, 64, 3))
                decoded += 1
            self.assertEqual(decoded, len(self.rows()))
        finally:
            capture.release()


if __name__ == '__main__':
    unittest.main()
