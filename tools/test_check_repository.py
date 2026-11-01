"""Offline regression checks for JSON, file selection and documentation links."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import check_repository as checker


class RepositoryChecksTests(unittest.TestCase):
    def test_normal_json_and_explicit_missing_values(self):
        self.assertEqual(checker.parse_json('{"position": null, "speed": 1.25}'),
                         {'position': None, 'speed': 1.25})

    def test_nonfinite_json_numbers_are_rejected(self):
        for value in ['NaN', 'Infinity', '-Infinity']:
            with self.subTest(value=value), self.assertRaises(ValueError):
                checker.parse_json('{"speed": ' + value + '}')

    def test_local_environments_and_generated_output_are_skipped(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name in checker.EXCLUDED:
                (root / name).mkdir()
                (root / name / 'not-source.py').write_text('invalid Python', encoding='utf-8')
            source = root / 'tools' / 'example.py'
            source.parent.mkdir()
            source.write_text('pass', encoding='utf-8')
            with patch.object(checker, 'ROOT', root):
                self.assertEqual(checker.repository_files(), [source])

    def test_local_links_include_spaces_and_ignore_code_examples(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            page = root / 'README.md'
            target = root / 'file with spaces.md'
            target.touch()
            content = '[file](file%20with%20spaces.md)\n```\n[example](not-a-file.md)\n```'
            with patch.object(checker, 'ROOT', root):
                self.assertEqual(checker.local_links(page, content), [])

    def test_unreadable_source_tree_fails_instead_of_silently_skipping_it(self):
        def failed_walk(root, *, onerror):
            onerror(PermissionError('Unreadable source directory'))

        with patch.object(checker.os, 'walk', side_effect=failed_walk):
            with self.assertRaises(PermissionError):
                checker.repository_files()

    def test_missing_local_link_is_reported(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with patch.object(checker, 'ROOT', root):
                issues = checker.local_links(root / 'README.md', '[missing](absent.md)')
            self.assertEqual(len(issues), 1)
            self.assertIn('absent.md', issues[0])


if __name__ == '__main__':
    unittest.main()
