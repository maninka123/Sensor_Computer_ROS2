#!/usr/bin/env python3
"""Tie a successful software acceptance run to the exact source/build/image."""
from pathlib import Path
import argparse
import datetime
import hashlib
import json
import subprocess

ROOT=Path(__file__).resolve().parents[2]
STAMP=ROOT/'.local-deploy/software-tests-passed'


def fingerprint():
    digest=hashlib.sha256()
    for directory in ['src','tools','tests','deploy','docker','install']:
        base=ROOT/'ros2'/directory
        if not base.is_dir():
            continue
        for path in sorted(base.rglob('*')):
            if '__pycache__' in path.parts or path.suffix=='.pyc' or not path.is_file():
                continue
            digest.update(str(path.relative_to(ROOT)).encode()+b'\0')
            digest.update(path.read_bytes())
    image=subprocess.check_output(['docker','image','inspect','-f','{{.Id}}',
                                   'sensor-computer:jazzy-sdk'],text=True).strip()
    digest.update(image.encode())
    return digest.hexdigest(),image


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('mode',choices=['fingerprint','write','check'])
    parser.add_argument('--expected')
    args=parser.parse_args()
    value,image=fingerprint()
    if args.mode=='fingerprint':
        print(value)
    elif args.mode=='write':
        if value!=args.expected:
            raise SystemExit('Source/build/image changed during verification; acceptance was not recorded.')
        STAMP.parent.mkdir(exist_ok=True)
        STAMP.write_text(json.dumps({'fingerprint':value,'image_id':image,
            'passed_at':datetime.datetime.now(datetime.timezone.utc).isoformat()},indent=2)+'\n')
    else:
        try:
            recorded=json.loads(STAMP.read_text())
        except (OSError,ValueError):
            raise SystemExit('No valid software acceptance result; run ros2/tools/verify.sh.')
        if recorded.get('fingerprint')!=value:
            raise SystemExit('Acceptance is stale after source/build/image changes; run ros2/tools/verify.sh.')
        print('PASS: acceptance matches current source, build and image.')


if __name__=='__main__':
    main()
