"""Motor-shaft units: radians, radians/second, newton-metres."""
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping
import json
import math
import numpy as np
import mujoco
from .build import ROOT, build, folded_angles

@dataclass(frozen=True)
class MotorCommand:
    mode: str
    value: float

def validate(commands, names, config):
    result={}
    for name, cmd in commands.items():
        if name not in names: raise KeyError(f'Unknown motor {name}; expected {names}')
        if cmd.mode not in ('velocity','torque'): raise ValueError('mode must be velocity or torque')
        if not math.isfinite(cmd.value): raise ValueError('Motor commands must be finite')
        limit=config['motor_speed_limit_rad_s' if cmd.mode=='velocity' else 'motor_torque_limit_nm']
        if abs(cmd.value)>limit: raise ValueError(f'{name}: {cmd.value} exceeds configured {limit}')
        result[name]=cmd
    return result

class Simulation:
    def __init__(self, model_path=None):
        path=Path(model_path or ROOT/'models/robot.xml')
        if not path.exists(): build()
        metadata=json.loads(path.with_name('manifest.json').read_text())
        self.fold_couplings=metadata.get('fold_couplings',[])
        self.config=metadata['config']; self.names=tuple(x['name'] for x in metadata['motors'])
        self.model=mujoco.MjModel.from_xml_path(str(path)); self.data=mujoco.MjData(self.model)
        self._ids={n:self.model.actuator(n).id for n in self.names}
        self._joints={n:self.model.joint(n) for n in self.names}
        self._commands={}; self._updated={}; self._initial=metadata['initial_fold']
        self.reset()

    def reset(self, flat=False, fold_angle=None):
        mujoco.mj_resetData(self.model,self.data)
        angles=folded_angles(fold_angle) if fold_angle is not None else ([0.,0.,0.] if flat else self._initial)
        for i,q in enumerate(angles,1): self.data.joint(f'fold_{i}').qpos[0]=q
        for coupling in self.fold_couplings:
            name=coupling['motor']; q=float(self.data.joint(coupling['fold_joint']).qpos[0])*coupling['sign']
            self.data.joint(name+'_output').qpos[0]=q
            self.data.joint(name).qpos[0]=q*self.config['gear_ratio']
            self.data.joint(name+'_idler').qpos[0]=-q*self.config['gear_ratio']
        mujoco.mj_forward(self.model,self.data)
        # Start just above the floor, accounting for every contact proxy after folding.
        if not self.config['fixed_base']:
            bottoms=[]
            for i in range(self.model.ngeom):
                if not (self.model.geom_conaffinity[i]&1) or self.model.geom_type[i]==mujoco.mjtGeom.mjGEOM_PLANE: continue
                rotation=self.data.geom_xmat[i].reshape(3,3); size=self.model.geom_size[i]
                kind=self.model.geom_type[i]
                if kind==mujoco.mjtGeom.mjGEOM_MESH:
                    mid=self.model.geom_dataid[i]
                    adr=self.model.mesh_vertadr[mid]; count=self.model.mesh_vertnum[mid]
                    verts=self.model.mesh_vert[adr:adr+count]
                    bottoms.append(float((verts@rotation[2]).min()+self.data.geom_xpos[i,2]))
                    continue
                if kind==mujoco.mjtGeom.mjGEOM_CYLINDER:
                    vertical=rotation[2,2]; extent=size[0]*np.sqrt(max(0,1-vertical**2))+size[1]*abs(vertical)
                elif kind==mujoco.mjtGeom.mjGEOM_BOX:
                    extent=np.abs(rotation[2])@size
                else: extent=size[0]+size[1]*abs(rotation[2,2])
                bottoms.append(self.data.geom_xpos[i,2]-extent)
            self.data.qpos[2]+=.002-min(bottoms)
            mujoco.mj_forward(self.model,self.data)
        self.stop()

    def command(self, commands: Mapping[str, MotorCommand]):
        checked=validate(commands,self.names,self.config)
        self._commands.update(checked)
        self._updated.update({name:self.data.time for name in checked})

    def set_velocity(self, name, rad_s): self.command({name:MotorCommand('velocity',rad_s)})
    def set_torque(self, name, nm): self.command({name:MotorCommand('torque',nm)})

    def stop(self):
        """Remove commanded torque; this coasts rather than brakes."""
        self._commands.clear(); self._updated.clear(); self.data.ctrl[:]=0

    def step(self, seconds=.01):
        if not math.isfinite(seconds) or seconds<=0: raise ValueError('seconds must be positive and finite')
        dt=self.model.opt.timestep
        steps=round(seconds/dt)
        if steps<1 or not math.isclose(steps*dt,seconds,rel_tol=1e-8,abs_tol=1e-12):
            raise ValueError('seconds must be a positive integer multiple of timestep')
        for _ in range(steps):
            self.data.ctrl[:]=0
            for n,cmd in list(self._commands.items()):
                if self.data.time-self._updated[n]>=self.config['command_timeout_s']: continue
                aid=self._ids[n]; speed=self.data.joint(n).qvel[0]
                torque=cmd.value if cmd.mode=='torque' else self.config['velocity_kp']*(cmd.value-speed)
                # Software overspeed limiter: permit braking but no further acceleration.
                if abs(speed)>=self.config['motor_speed_limit_rad_s'] and torque*speed>0: torque=0
                self.data.ctrl[aid]=np.clip(torque,-self.config['motor_torque_limit_nm'],self.config['motor_torque_limit_nm'])
            mujoco.mj_step(self.model,self.data)
            if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
                self.stop(); raise RuntimeError('Simulation diverged')
        return self.read()

    def read(self):
        return {'time':float(self.data.time),'motors':{n:{
            'position_rad':float(self.data.joint(n).qpos[0]),
            'velocity_rad_s':float(self.data.joint(n).qvel[0]),
            'wheel_velocity_rad_s':float(self.data.joint(n+'_output').qvel[0]),
            'commanded_torque_nm':float(self.data.ctrl[self._ids[n]])} for n in self.names},
            'fold_rad':[float(self.data.joint(f'fold_{i}').qpos[0]) for i in range(1,4)],
            'closure_error_m':float(np.linalg.norm(self.data.site('closure_0').xpos-self.data.site('closure_3').xpos))}
