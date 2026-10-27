"""Operator velocity commands; no robot channels or motor publishers."""
import os
import select
import struct
import sys

import numpy as np


class UnitreeJoystickCommands:
    """Decode the SDK's wireless_remote[40], then map sticks to body vx/vy/wz."""
    def __init__(self,scale=(.30,.15,.40),deadzone=.08):
        self.scale=np.asarray(scale,dtype=float)
        if self.scale.shape!=(3,) or not np.isfinite(self.scale).all() or np.any(self.scale<0):
            raise ValueError('Joystick scales must be three finite nonnegative numbers')
        if not np.isfinite(deadzone) or not 0<=deadzone<1:
            raise ValueError('Joystick deadzone must be finite and within [0, 1)')
        self.deadzone=float(deadzone)
        self.axes=np.zeros(4)  # lx, ly, rx, ry

    def update(self,remote):
        if remote is None or len(remote)!=40:
            raise ValueError('Missing or malformed Unitree wireless_remote[40]')
        data=bytes(remote)
        self.axes=np.array([struct.unpack_from('<f',data,offset)[0] for offset in (4,20,8,12)])
        if not np.isfinite(self.axes).all() or np.any(np.abs(self.axes)>1):
            raise ValueError('Invalid Unitree joystick axes: expected finite values in [-1, 1]')
        lx,ly,rx,_=self.axes
        axes=np.array([ly,-lx,-rx])  # x forward, y left, positive yaw left.
        shaped=np.sign(axes)*np.maximum(np.abs(axes)-self.deadzone,0)/(1-self.deadzone)
        return shaped*self.scale


class TerminalCommands:
    def __init__(self,limits=(.45,.15,.6)):
        self.limits=None if limits is None else np.asarray(limits,dtype=float)
        self.command=np.zeros(3);self.quit=False;self.pending=b''

    def accept(self,line):
        line=line.strip().lower()
        if line in ('quit','exit'):self.command[:]=0;self.quit=True
        elif line in ('stop','x'):self.command[:]=0
        else:
            try:values=np.array([float(x) for x in line.replace(',',' ').split()])
            except ValueError:raise ValueError('Enter vx vy wz, stop, or quit') from None
            if values.shape!=(3,) or not np.isfinite(values).all():
                raise ValueError('Enter three finite numbers: vx vy wz')
            if self.limits is not None and np.any(np.abs(values)>self.limits):
                raise ValueError('Command range: |vx| <= 0.45 m/s, |vy| <= 0.15 m/s, |wz| <= 0.6 rad/s')
            self.command=values
        return self.command.copy()

    def poll(self):
        if select.select([sys.stdin],[],[],0)[0]:
            # Read the descriptor directly: TextIO.readline can prefetch a
            # pasted 'stop' line that select() can no longer see next cycle.
            chunk=os.read(sys.stdin.fileno(),4096)
            if not chunk:self.quit=True;self.command[:]=0
            else:
                self.pending+=chunk
                while b'\n' in self.pending:
                    line,self.pending=self.pending.split(b'\n',1)
                    try:print('[command] '+str(self.accept(line.decode('utf-8',errors='replace')).tolist()),flush=True)
                    except ValueError as exc:print('[command] '+str(exc),flush=True)
                    if self.quit:break
                if len(self.pending)>4096:
                    self.pending=b'';print('[command] Input line too long',flush=True)
        return self.command.copy()
