#!/usr/bin/env python3
"""Offline MuJoCo check of the real navigation gate, loopback link and policy adapter."""
import argparse
import csv
import json
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
os.environ.setdefault('MUJOCO_GL', 'egl')

import numpy as np
from navigation.homie_deploy import HomieDeployCommand
from navigation.homie_mujoco import MuJoCoSimulation, quaternion_rpy
from navigation.homie_nav2_transport import HomieNavClient, HomieNavCommands
from navigation.homie_protocol import NAV2_PROTOCOL
from navigation.odom_velocity import body_twist_to_yaw
from navigation.real_g1_core import VelocityGate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trials', type=int, default=3)
    parser.add_argument('--protocol', choices=('low-speed','full-range','deployment'),default='low-speed')
    parser.add_argument('--output', type=Path, default=ROOT/'.cache-g1/homie-nav2-mujoco')
    args = parser.parse_args()
    if args.trials < 1:parser.error('--trials must be positive')
    args.output.mkdir(parents=True, exist_ok=True)
    sim = MuJoCoSimulation('official27', feedback=False, execution='position', physics_dt=.002)
    control = HomieDeployCommand(True, yaw_feedforward=.4, limits=(.6,.6,.6),navigation=True)
    protocol=NAV2_PROTOCOL
    if args.protocol in ('full-range','deployment'):
        protocol=[('settle',5.,(0.,0.,0.)),('stand',10.,(0.,0.,0.))]
        vx,vy,wz=VelocityGate().limits if args.protocol=='deployment' else (.6,.6,.6)
        for phase,command in (
            ('forward',(vx,0.,0.)),('lateral_positive',(0.,vy,0.)),
            ('lateral_negative',(0.,-vy,0.)),('yaw_positive',(0.,0.,wz)),
            ('yaw_negative',(0.,0.,-wz)),('mixed',(.5*vx,.5*vy,.5*wz))):
            protocol.extend([(phase,15.,command),('stop_'+phase,8.,(0.,0.,0.))])
    result = dict(hardware_command_count=0, policy_dt_s=.02, navigation_link_dt_s=.1,
                  configuration=sim.metadata,
                  feedback_source='MuJoCo body twist -> same ROS projection -> loopback UDP',
                  yaw_feedforward_rad_s=.4, feedback_limit=control.feedback.limit.tolist(),
                  protocol=args.protocol,acceleration=control.feedback.acceleration.tolist(),smooth_stop=control.feedback.smooth_stop,
                  trials=[], failures=[])
    started = time.monotonic()
    with (args.output/'samples.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['trial', 'time', 'phase', 'cmd_vx', 'cmd_vy', 'cmd_wz',
                         'gate_vx', 'gate_vy', 'gate_wz', 'policy_vx', 'policy_vy', 'policy_wz',
                         'vx', 'vy', 'wz', 'root_z', 'torso_roll', 'torso_pitch'])
        for trial in range(args.trials):
            sim.reset(trial)
            control.reset()
            gate = VelocityGate(permitted=True)
            gate.arm(True, 0.)
            worker = HomieNavCommands(0, require_feedback=True)
            client = HomieNavClient(worker.port)
            client.send()  # Disabled packet claims this loopback session.
            worker.poll()
            step = 0
            phases = []
            failed = False
            try:
                for phase, seconds, command in protocol:
                    samples = []
                    print(f'[nav2-mujoco] trial={trial} phase={phase} command={command}', flush=True)
                    for k in range(round(seconds/.02)):
                        if step % 5 == 0:
                            rotation = sim.data.xmat[sim.pelvis].reshape(3, 3)
                            body_linear = rotation.T @ sim.data.qvel[:3]
                            q = sim.data.xquat[sim.pelvis]
                            measured = body_twist_to_yaw(q[[1, 2, 3, 0]], body_linear, sim.data.qvel[3:6])
                            np.testing.assert_allclose(measured, sim.velocity(), atol=1e-12)
                            gate.update(command, step*.02)
                            gated = gate.step(step*.02, True)
                            client.send(gated, enabled=True, measured_velocity=measured,
                                        measurement_stamp=time.monotonic())
                        worker.poll(active=True, ready=True, phase='homie_active')
                        client.poll()
                        adapted = control.update(worker.command, worker.measured_velocity(),now=step*.02)
                        fell = sim.step(adapted)
                        velocity = sim.velocity()
                        roll, pitch, _ = quaternion_rpy(sim.data.xquat[sim.torso])
                        writer.writerow([trial, (step+1)*.02, phase, *command, *gate.output,
                                         *adapted, *velocity, sim.data.qpos[2], roll, pitch])
                        samples.append(velocity.copy())
                        step += 1
                        if fell:
                            result['failures'].append(dict(trial=trial, phase=phase, time_s=step*.02,
                                                           root_z=float(sim.data.qpos[2])))
                            failed = True
                            break
                    tail = np.array(samples[-200:])
                    phases.append(dict(phase=phase, command=command, tail_mean=tail.mean(axis=0).tolist()))
                    stream.flush()
                    if failed:break
            finally:
                client.close()
                worker.close()
            result['trials'].append(dict(trial=trial, completed=not failed, simulated_seconds=step*.02, phases=phases))
    result['simulated_seconds'] = sum(t['simulated_seconds'] for t in result['trials'])
    result['wall_seconds'] = time.monotonic()-started
    (args.output/'result.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k: v for k, v in result.items() if k not in ('trials', 'configuration')}), flush=True)
    return int(bool(result['failures']))


if __name__ == '__main__':raise SystemExit(main())
