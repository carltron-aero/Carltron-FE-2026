"""Offline checks for the included numerical data and its manifest."""
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

import render_figures as figures


class DataValidationTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)

    def dataset(self, body='{"values": [1, 2]}'):
        path = self.root / 'example.json'
        path.write_text(body, encoding='utf-8')
        manifest = {'files': {'example.json': hashlib.sha256(path.read_bytes()).hexdigest()}}
        self.write_manifest(manifest)
        return manifest

    def write_manifest(self, manifest):
        (self.root / 'manifest.json').write_text(json.dumps(manifest), encoding='utf-8')

    def verify(self):
        with patch.object(figures, 'DATA', self.root):
            return figures.verify_data()

    def test_included_data_matches_its_manifest(self):
        self.assertEqual(len(figures.verify_data()['files']), 16)

    def test_git_preserves_the_hashed_export_bytes(self):
        repo = Path(__file__).resolve().parents[2]
        for name in figures.verify_data()['files']:
            with self.subTest(file=name):
                relative = f'tools/experiments/data/{name}'
                raw = subprocess.check_output(
                    ['git', '-C', str(repo), 'hash-object', '--no-filters', '--', relative])
                stored = subprocess.check_output(
                    ['git', '-C', str(repo), 'hash-object', f'--path={relative}', '--', relative])
                self.assertEqual(raw, stored, 'Git must preserve the bytes covered by the manifest')

    def test_unchanged_fixture_passes(self):
        manifest = self.dataset()
        self.assertEqual(self.verify(), manifest)

    def test_changed_data_is_rejected(self):
        self.dataset()
        (self.root / 'example.json').write_text('{"values": [3]}', encoding='utf-8')
        with self.assertRaisesRegex(ValueError, 'hash differs'):
            self.verify()

    def test_missing_data_is_rejected(self):
        self.dataset()
        (self.root / 'example.json').unlink()
        with self.assertRaises(FileNotFoundError):
            self.verify()

    def test_unlisted_data_is_rejected(self):
        self.dataset()
        (self.root / 'extra.json').write_text('{}', encoding='utf-8')
        with self.assertRaisesRegex(ValueError, 'manifest differ'):
            self.verify()

    def test_paths_are_rejected_instead_of_reading_outside_data(self):
        for name in ['../outside.json', '..\\outside.json', '/absolute.json', 'C:\\outside.json', 'manifest.json']:
            with self.subTest(name=name):
                self.write_manifest({'files': {name: '0' * 64}})
                with self.assertRaisesRegex(ValueError, 'Invalid data filename'):
                    self.verify()

    def test_invalid_hash_is_rejected(self):
        manifest = self.dataset()
        for value in ['not-a-hash', 123, None]:
            with self.subTest(value=value):
                manifest['files']['example.json'] = value
                self.write_manifest(manifest)
                with self.assertRaisesRegex(ValueError, 'Invalid SHA-256'):
                    self.verify()

    def test_nonfinite_data_is_rejected_even_when_the_hash_matches(self):
        for value in ['NaN', 'Infinity', '-Infinity']:
            with self.subTest(value=value):
                self.dataset('{"value": ' + value + '}')
                with self.assertRaisesRegex(ValueError, 'Non-finite'):
                    self.verify()

    def test_empty_manifest_is_rejected(self):
        self.write_manifest({'files': {}})
        with self.assertRaisesRegex(ValueError, 'at least one'):
            self.verify()


if __name__ == '__main__':
    unittest.main()
