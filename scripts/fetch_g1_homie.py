#!/usr/bin/env python3
"""Fetch and audit pinned OpenHomie weights, source contract, and 27-DoF assets."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from navigation.homie import CHECKPOINT_SHA,COMMIT,MODEL_RELATIVE,model_joints


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--proxy');args=parser.parse_args()
    destination=ROOT/'.cache-g1/OpenHomie'
    git=['git']+(['-c',f'http.proxy={args.proxy}'] if args.proxy else [])
    if not destination.exists():
        temporary=destination.with_name(destination.name+'.partial')
        if temporary.exists():raise RuntimeError('Inspect previous partial checkout: '+str(temporary))
        temporary.parent.mkdir(parents=True,exist_ok=True)
        subprocess.run([*git,'init',str(temporary)],check=True)
        subprocess.run([*git,'-C',str(temporary),'remote','add','origin','https://github.com/InternRobotics/OpenHomie.git'],check=True)
        subprocess.run([*git,'-C',str(temporary),'fetch','--depth','1','origin',COMMIT],check=True)
        subprocess.run([*git,'-C',str(temporary),'checkout','--detach','FETCH_HEAD'],check=True)
        temporary.rename(destination)
    commit=subprocess.check_output([*git,'-C',str(destination),'rev-parse','HEAD'],text=True).strip()
    if commit!=COMMIT:raise RuntimeError('Unexpected OpenHomie commit; checkout preserved: '+commit)
    dirty=subprocess.check_output([*git,'-C',str(destination),'status','--porcelain','--untracked-files=no'],text=True)
    if dirty.strip():raise RuntimeError('OpenHomie tracked files were modified; checkout preserved')
    asset=destination/MODEL_RELATIVE;xml,_=model_joints(asset)
    files={asset,destination/'HomieDeploy/deploy.onnx',destination/'HomieDeploy/unitree_sdk2/g1_control.cpp',
        destination/'HomieDeploy/g1_gym_deploy/envs/lcm_agent.py',
        destination/'HomieDeploy/g1_gym_deploy/envs/history_wrapper.py'}
    for mesh in xml.findall('.//mesh'):
        path=(asset.parent/mesh.attrib['filename']).resolve()
        if not path.is_relative_to(destination.resolve()):raise RuntimeError('Mesh escapes source checkout')
        files.add(path)
    hashes={str(p.relative_to(destination)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(files)}
    if hashes['HomieDeploy/deploy.onnx']!=CHECKPOINT_SHA:raise RuntimeError('OpenHomie policy checksum mismatch')
    manifest={'commit':commit,'sha256':hashes,'movable_joints':27,'actions':12,'frame':76,'history':6,
        'source':'https://github.com/InternRobotics/OpenHomie','license':'Upstream LICENSE retained in source checkout'}
    (destination/'integration-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(f'OpenHomie ready: {destination}\nCommit: {commit}\nONNX SHA-256: {CHECKPOINT_SHA}\n27 joints / 6x76 observations / 12 leg actions')


if __name__=='__main__':main()
