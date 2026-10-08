#!/usr/bin/env python3
"""Verify and restore the complete analysis snapshot using Python's stdlib."""

import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import shutil
import tempfile
from zipfile import ZipFile

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]


def sha256_stream(stream):
    digest = hashlib.sha256()
    for block in iter(lambda: stream.read(1024 * 1024), b''):
        digest.update(block)
    return digest.hexdigest()


def sha256_file(path):
    with path.open('rb') as stream:
        return sha256_stream(stream)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--destination', type=Path, default=REPO,
                        help='Repository root to restore into (default: this clone).')
    parser.add_argument('--verify-only', action='store_true', help='Check archive integrity without writing files.')
    parser.add_argument('--overwrite', action='store_true', help='Explicitly replace different existing result files.')
    parser.add_argument('--snapshot', choices=('analysis', 'postprocessing', 'neutral_to_grasp'), default='analysis',
                        help='Original analysis, later postprocessing, or natural-to-grasp experiment results.')
    args = parser.parse_args()
    manifest = json.loads((HERE/f'{args.snapshot}_manifest.json').read_text(encoding='utf-8'))
    archive_path = HERE/f'{args.snapshot}_results.zip'
    if sha256_file(archive_path) != manifest['archive_sha256']:
        raise SystemExit('Archive SHA-256 mismatch; fetch the snapshot again.')
    root = args.destination.expanduser().resolve()
    expected = {entry['path']: entry for entry in manifest['files']}
    pending, unchanged, conflicts = [], [], []
    with ZipFile(archive_path) as archive:
        names = archive.namelist()
        if len(names) != len(set(names)) or set(names) != set(expected):
            raise SystemExit('Archive contents differ from the manifest.')
        for name in names:
            relative = PurePosixPath(name)
            if (relative.is_absolute() or '..' in relative.parts or
                    relative.parts[:2] not in (('results', 'solver_only'), ('results', 'trajectory_audit'))):
                raise SystemExit(f'Unexpected snapshot path: {name}')
            target = root.joinpath(*relative.parts)
            if not target.resolve().is_relative_to(root):
                raise SystemExit(f'Restore path escapes destination: {name}')
            info, entry = archive.getinfo(name), expected[name]
            if info.file_size != entry['bytes']:
                raise SystemExit(f'File size mismatch: {name}')
            with archive.open(name) as stream:
                if sha256_stream(stream) != entry['sha256']:
                    raise SystemExit(f'File SHA-256 mismatch: {name}')
            if args.verify_only:
                continue
            if target.exists():
                if target.is_file() and sha256_file(target) == entry['sha256']:
                    unchanged.append(name)
                    continue
                if not args.overwrite or not target.is_file():
                    conflicts.append(name)
                    continue
            pending.append((name, target))
        if args.verify_only:
            print(f'Verified {len(names)} files ({manifest["uncompressed_bytes"]:,} bytes); no files written.')
            return
        if conflicts:
            raise SystemExit('Existing files differ; nothing restored. Use another --destination or explicitly --overwrite:\n' + '\n'.join(conflicts))
        for name, target in pending:
            target.parent.mkdir(parents=True, exist_ok=True)
            temp_path = None
            try:
                with tempfile.NamedTemporaryFile(dir=target.parent, delete=False) as output:
                    temp_path = Path(output.name)
                    with archive.open(name) as source:
                        shutil.copyfileobj(source, output)
                temp_path.replace(target)
            finally:
                if temp_path is not None and temp_path.exists():
                    temp_path.unlink()
    print(f'Restored {len(pending)} files; {len(unchanged)} identical files already present. Destination: {root}')


if __name__ == '__main__':
    main()
