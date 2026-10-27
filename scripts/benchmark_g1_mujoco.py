#!/usr/bin/env python3
"""Run the same OpenHomie ONNX and command protocol in MuJoCo. No robot transport."""
import argparse
from contextlib import nullcontext
import csv
import json
import os
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
os.environ.setdefault('MUJOCO_GL','egl')

import mujoco
import numpy as np
from navigation.homie_mujoco import MuJoCoSimulation,DisplaySnapshot,quaternion_rpy
from navigation.homie_protocol import PROTOCOL,NAV2_PROTOCOL
from navigation.locomotion_commands import TerminalCommands

COLUMNS=['trial','time','phase_time','phase','cmd_vx','cmd_vy','cmd_wz','x','y','root_z','yaw',
         'vx','vy','wz','torso_roll','torso_pitch','policy_vx','policy_vy','policy_wz','engine_time']


def run(args,feedback):
    sim=MuJoCoSimulation(args.model,feedback,args.execution,args.physics_dt,args.gains)
    if sim.feedback is not None:sim.feedback.feedforward[2]=args.yaw_feedforward
    output=args.output;output.mkdir(parents=True,exist_ok=True)
    name=sim.name
    custom=args.interactive or args.command is not None
    protocol=([('settle',5.,(0.,0.,0.)),
               ('interactive' if args.interactive else 'constant',args.seconds,tuple(args.command or [0.,0.,0.]))]
              if custom else NAV2_PROTOCOL if args.protocol=='nav2' else PROTOCOL)
    terminal=TerminalCommands() if args.interactive else None
    metadata={**sim.metadata,'protocol':[{'phase':n,'seconds':s,'command':c} for n,s,c in protocol],
              'command_mode':'terminal' if args.interactive else 'constant' if custom else 'benchmark',
              'protocol_name':args.protocol,
              'yaw_feedforward_rad_s':args.yaw_feedforward if feedback else 0.,
              'viewer_state_isolated':args.viewer,
              'trial_seeds':list(range(42,42+args.trials)),'initial_joint_perturbation_rad':[-.005,.005],
              'fall_criteria':'pelvis z < 0.40m OR abs(torso roll/pitch) > 1 rad; no reset within trial'}
    (output/f'{name}-metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
    status={'policy':name,'completed_trials':0,'failures':[],'finished':False,'hardware_command_count':0}
    renderer=video=None
    start=time.monotonic()
    if args.viewer:
        from mujoco import viewer as mj_viewer
        sim.reset(0)
        display=DisplaySnapshot(sim)
        view_context=mj_viewer.launch_passive(display.model,display.data)
    else:view_context=nullcontext(None)
    try:
        if args.video and (feedback or args.feedback=='raw'):
            import imageio.v2 as imageio
            renderer=mujoco.Renderer(sim.model,height=480,width=640)
            video=imageio.get_writer(str(output/f'{name}.mp4'),fps=50/3,codec='libx264',quality=7)
        camera=mujoco.MjvCamera();camera.distance=2.8;camera.azimuth=135;camera.elevation=-20
        with view_context as viewer,(output/f'{name}-samples.csv').open('w',newline='') as stream:
            writer=csv.writer(stream);writer.writerow(COLUMNS)
            for trial in range(args.trials):
                sim.reset(trial);step=0;failed=False;trial_wall=time.monotonic()
                for phase,seconds,command in protocol:
                    print(f'[mujoco] {name} trial={trial} phase={phase} command={command}',flush=True)
                    if terminal is not None and phase=='interactive':
                        print('[command] Enter vx vy wz in THIS terminal, then Enter. '
                              'Example: 0.3 0 0 | 0 0 0.4 | stop | quit. Command persists until changed.',flush=True)
                    for k in range(round(seconds/.02)):
                        if viewer is not None and not viewer.is_running():raise RuntimeError('Viewer closed before trial completion')
                        if terminal is not None and phase=='interactive':
                            command=terminal.poll()
                            if terminal.quit:break
                        fell=sim.step(command)
                        q=sim.data.qpos;roll,pitch,_=quaternion_rpy(sim.data.xquat[sim.torso])
                        writer.writerow([trial,(step+1)*.02,(k+1)*.02,phase,*command,*q[:3],
                            quaternion_rpy(sim.data.xquat[sim.pelvis])[2],*sim.velocity(),roll,pitch,*sim.last_policy_command,sim.data.time])
                        if video is not None and trial==0 and step%3==0:
                            from PIL import Image,ImageDraw
                            camera.lookat[:]=sim.data.xpos[sim.pelvis];camera.lookat[2]=.7
                            renderer.update_scene(sim.data,camera=camera)
                            frame=Image.fromarray(renderer.render());draw=ImageDraw.Draw(frame)
                            draw.rectangle((0,0,640,48),fill=(20,24,30))
                            draw.text((10,6),f'{name} | {phase} | t={(step+1)*.02:.2f}s',fill='white')
                            draw.text((10,26),f'vx={command[0]:.2f} m/s   wz={command[2]:.2f} rad/s',fill='white')
                            video.append_data(np.asarray(frame))
                        step+=1
                        if viewer is not None:
                            with viewer.lock():
                                display.update()
                                viewer.cam.lookat[:]=sim.data.xpos[sim.pelvis]
                            viewer.sync()
                        if viewer is not None or terminal is not None:
                            time.sleep(max(0,step*.02-(time.monotonic()-trial_wall)))
                        if fell:
                            status['failures'].append({'trial':trial,'phase':phase,'time_s':step*.02,
                                'root_z':float(q[2]),'torso_roll_pitch':[float(roll),float(pitch)]})
                            failed=True;break
                    stream.flush()
                    if failed or (terminal is not None and terminal.quit):break
                if not failed and not (terminal is not None and terminal.quit):status['completed_trials']+=1
                if terminal is not None and terminal.quit:
                    status['operator_exit']=True;break
        status['finished']=True
    except Exception as exc:
        status['failures'].append({'error':str(exc)})
        raise
    finally:
        if video is not None:video.close()
        if renderer is not None:renderer.close()
        status['wall_seconds']=time.monotonic()-start
        (output/f'{name}-status.json').write_text(json.dumps(status,indent=2)+'\n')
        (output/f'{name}-execution.json').write_text(json.dumps(sim.diagnostics_json(),indent=2)+'\n')
        print('[mujoco] '+json.dumps(status),flush=True)
    return not status['failures']


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model',choices=('official27','homie27'),default='official27')
    parser.add_argument('--gains',choices=('deploy','training'),default='deploy')
    parser.add_argument('--feedback',choices=('raw','closed-loop','both'),default='both')
    parser.add_argument('--protocol',choices=('standard','nav2'),default='standard',
                        help='Automatic test velocities: original benchmark or current real-navigation envelope')
    parser.add_argument('--yaw-feedforward',type=float,default=0.,
                        help='Optional yaw dead-zone feedforward in rad/s, before the PI output limit')
    parser.add_argument('--execution',choices=('author','position'),default='author')
    parser.add_argument('--trials',type=int,default=3)
    parser.add_argument('--physics-dt',type=float,default=.005)
    parser.add_argument('--output',type=Path,default=ROOT/'.cache-g1/homie-mujoco-validation')
    parser.add_argument('--video',action='store_true');parser.add_argument('--viewer',action='store_true')
    parser.add_argument('--interactive',action='store_true',help='Enter vx vy wz / stop / quit in the launching terminal')
    parser.add_argument('--command',type=float,nargs=3,metavar=('VX','VY','WZ'),help='Hold this velocity after settling for 5s')
    parser.add_argument('--seconds',type=float,default=120.,help='Duration of constant or interactive mode, excluding 5s settling')
    args=parser.parse_args()
    if args.trials<1:parser.error('--trials must be positive')
    if not np.isfinite(args.yaw_feedforward) or args.yaw_feedforward<0:
        parser.error('--yaw-feedforward must be finite and nonnegative')
    if args.interactive and args.command is not None:parser.error('Choose --interactive or --command')
    if not np.isfinite(args.seconds) or args.seconds<=0:parser.error('--seconds must be positive and finite')
    if (args.interactive or args.command is not None) and (args.trials!=1 or args.feedback=='both'):
        parser.error('Manual modes require --trials 1 and --feedback raw or closed-loop')
    if args.interactive and not sys.stdin.isatty():parser.error('--interactive requires a terminal')
    if args.command is not None:
        try:TerminalCommands().accept(' '.join(map(str,args.command)))
        except ValueError as exc:parser.error(str(exc))
    modes=[False,True] if args.feedback=='both' else [args.feedback=='closed-loop']
    passed=True
    for mode in modes:passed=run(args,mode) and passed
    return 0 if passed else 1


if __name__=='__main__':raise SystemExit(main())
