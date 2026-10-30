"""Tests for safe, non-destructive extraction of the release evidence."""
from pathlib import Path
import stat
import tempfile
import unittest
import zipfile
from download_data import extract_verified


class EvidenceExtractionTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix='caster-extraction-test-')
        self.root = Path(self.directory.name)
        self.archive = self.root/'sample.zip'
        self.destination = self.root/'data'

    def tearDown(self):
        self.directory.cleanup()

    def write_zip(self, name='outputs/report.txt', content=b'original evidence'):
        with zipfile.ZipFile(self.archive, 'w') as z:
            z.writestr(name, content)

    def test_preserves_bytes_and_is_idempotent(self):
        self.write_zip()
        extract_verified(self.archive, self.destination)
        extract_verified(self.archive, self.destination)
        self.assertEqual((self.destination/'outputs/report.txt').read_bytes(), b'original evidence')

    def test_refuses_to_replace_changed_evidence(self):
        self.write_zip()
        extract_verified(self.archive, self.destination)
        target = self.destination/'outputs/report.txt'
        target.write_bytes(b'local investigation')
        with self.assertRaises(FileExistsError):
            extract_verified(self.archive, self.destination)
        self.assertEqual(target.read_bytes(), b'local investigation')

    def test_parent_traversal_rejected(self):
        self.write_zip('../escape.txt')
        with self.assertRaises(ValueError):
            extract_verified(self.archive, self.destination)
        self.assertFalse((self.root/'escape.txt').exists())

    def test_absolute_and_windows_paths_rejected(self):
        for path in ('/tmp/escape.txt', 'C:/escape.txt', 'outputs\\escape.txt'):
            with self.subTest(path=path):
                self.write_zip(path)
                with self.assertRaises(ValueError):
                    extract_verified(self.archive, self.destination)

    def test_symlink_entry_rejected(self):
        info = zipfile.ZipInfo('outputs/link')
        info.create_system = 3
        info.external_attr = (stat.S_IFLNK | 0o777) << 16
        with zipfile.ZipFile(self.archive, 'w') as z:
            z.writestr(info, '../../escape')
        with self.assertRaises(ValueError):
            extract_verified(self.archive, self.destination)

    def test_existing_destination_symlink_cannot_escape(self):
        self.write_zip()
        outside = self.root/'outside'
        outside.mkdir()
        self.destination.mkdir()
        try:
            (self.destination/'outputs').symlink_to(outside, target_is_directory=True)
        except OSError as exc:
            self.skipTest(f'Symlinks unavailable: {exc}')
        with self.assertRaises(ValueError):
            extract_verified(self.archive, self.destination)
        self.assertFalse((outside/'report.txt').exists())


if __name__=='__main__':
    unittest.main()
