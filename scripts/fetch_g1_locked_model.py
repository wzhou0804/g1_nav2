#!/usr/bin/env python3
"""Download the pinned official mode_machine=6 G1 URDF and its mesh dependencies."""
import argparse
import concurrent.futures
import hashlib
import json
from pathlib import Path
import urllib.request
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
COMMIT = '15ebe53f4cd77e7d745ae83b9d7ff90e685e8998'
BASE = f'https://raw.githubusercontent.com/unitreerobotics/unitree_ros/{COMMIT}/robots/g1_description/'
URDF = 'g1_29dof_lock_waist_rev_1_0.urdf'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'.cache-g1/locked-g1/source')
    parser.add_argument('--proxy', help='Optional proxy used only for these downloads')
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    previous_path = output/'manifest.json'
    previous = json.loads(previous_path.read_text()) if previous_path.exists() else None
    if previous is not None and previous['commit'] != COMMIT:
        raise RuntimeError('Existing model manifest uses a different commit; choose a new output directory')
    opener = urllib.request.build_opener(urllib.request.ProxyHandler(
        {'http': args.proxy, 'https': args.proxy} if args.proxy else {}))

    def fetch(name):
        path = output/name
        if not path.exists():
            with opener.open(BASE+name, timeout=120) as response:
                content = response.read()
            path.parent.mkdir(parents=True, exist_ok=True)
            temporary = path.with_suffix(path.suffix+'.partial')
            temporary.write_bytes(content)
            temporary.replace(path)
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        if previous is not None and name in previous['sha256'] and digest != previous['sha256'][name]:
            raise RuntimeError(f'Cached official model dependency checksum mismatch: {name}')
        return name, digest

    hashes = dict([fetch(URDF)])
    robot = ET.parse(output/URDF).getroot()
    meshes = sorted({item.attrib['filename'] for item in robot.iter('mesh')})
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        hashes.update(pool.map(fetch, meshes))
    joints = {j.attrib['name']: j for j in robot.findall('joint')}
    assert sum(j.attrib['type']=='revolute' for j in joints.values()) == 27
    locks = {name: {'type': joints[name].attrib['type'],
                    'origin': joints[name].find('origin').attrib}
             for name in ('waist_roll_joint', 'waist_pitch_joint')}
    assert all(value['type']=='fixed' for value in locks.values())
    manifest = {'upstream': 'https://github.com/unitreerobotics/unitree_ros',
                'commit': COMMIT, 'base_url': BASE, 'urdf': URDF,
                'movable_joints': 27, 'fixed_waist': locks, 'meshes': len(meshes),
                'urdf_total_mass_kg': sum(float(m.attrib['value']) for m in robot.iter('mass')),
                'sha256': hashes}
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps({k: v for k, v in manifest.items() if k!='sha256'}, indent=2))


if __name__ == '__main__':
    main()
