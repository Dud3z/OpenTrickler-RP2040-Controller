"""
Collects the firmware images of a CI build into a folder with a manifest.json
for the web updater (src/html/ai_trickle.html). CI publishes the folder to the
'firmware' branch, from where the updater downloads it.

Usage

    python scripts/make_firmware_manifest.py --out publish
"""

import argparse
import datetime
import hashlib
import json
import os
import shutil
import subprocess


FILES = {
    # name in the published folder: path in the build tree
    'pico2_w.bin': 'build_pico2_w/app.bin',
    'pico2_w.uf2': 'build_pico2_w/app.uf2',
    'pico2_w_partition_table.uf2': 'build_pico2_w/pt.uf2',
    'pico_w.uf2': 'build_pico_w/app.uf2',
}


def git(*args):
    return subprocess.check_output(['git', *args], text=True).strip()


def sha256(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as fp:
        for chunk in iter(lambda: fp.read(65536), b''):
            digest.update(chunk)
    return digest.hexdigest()


def main(out):
    os.makedirs(out, exist_ok=True)

    files = {}
    for name, source in FILES.items():
        if not os.path.exists(source):
            continue
        shutil.copyfile(source, os.path.join(out, name))
        files[name] = {'size': os.path.getsize(source), 'sha256': sha256(source)}

    manifest = {
        'build': int(git('rev-list', '--count', 'HEAD')),
        'commit': git('rev-parse', 'HEAD'),
        'version': git('describe', '--tags', '--long', '--always'),
        'date': datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='seconds'),
        'notes': git('log', '-15', '--format=%h %s').splitlines(),
        'files': files,
    }

    with open(os.path.join(out, 'manifest.json'), 'w') as fp:
        json.dump(manifest, fp, indent=2)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    main(args.out)
