import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'../')))

import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import differential_evolution

import data

class RotSpringTorque(chrono.TorqueFunctor):
    def __init__(self, k, b):
        super(RotSpringTorque, self).__init__()
        self.k = k
        self.b = b

    def __call__(self,time,angle,vel,link):
        torque = -self.k*angle-self.b*vel
        return torque

chrono.SetChronoDataPath('./chrono_data/')
PI = np.pi
DEG_2_RAD = np.pi/180
SREVO_OFFSET = -(150+90)*DEG_2_RAD

tfinal=1
step=2e-4

def read(file_name):
    exp_data = data.read(file_name)
    exp_data['pos_offset'] = []
    exp_data['t_offset'] = []

    for p in exp_data['pos']:
        exp_data['pos_offset'].append(-PI-(p+SREVO_OFFSET))

    # Get start and end angle
    pos_i = np.average(exp_data['pos_offset'][:10])
    pos_f = np.average(exp_data['pos_offset'][-10:])

    # Get start time
    t_offset = 0
    for i in range(len(exp_data['pos_offset'])):
        p = np.average(exp_data['pos_offset'][i:i+10])
        if p-pos_i < -0.01:
            t_offset = exp_data['t'][i]
            break
    # Offset time
    for t in exp_data['t']:
        exp_data['t_offset'].append(t-t_offset)

    return exp_data

def run(b,I,m):
    d = read('data/freefall_{:d}.csv'.format(int(m*1000)))

    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0,0,-9.81)) # No gravity

    ground = chrono.ChBodyEasyBox(0.01,0.01,0.01,1000,False)
    ground.SetPos(chrono.ChVectorD(0,0,0))
    ground.SetRot(chrono.Q_from_AngZ(0))
    ground.SetBodyFixed(True)
    system.Add(ground)

    gear = chrono.ChBody()
    gear.SetPos(chrono.ChVectorD(0,0,0))
    gear.SetRot(chrono.QUNIT)
    gear.SetMass(0.001)
    gear.SetInertiaXX(chrono.ChVectorD(I,I,I))
    system.Add(gear)
    # print(gear.GetInertiaXX())
    # print(gear.GetMass())

    r = 0.018 # servo arm
    load_pos = chrono.ChVectorD(0,r*np.cos(d['pos_offset'][0]),r*np.sin(d['pos_offset'][0]))
    load = chrono.ChBody()
    load.SetPos(load_pos)
    load.SetRot(chrono.QUNIT)
    load.SetMass(m)
    load.SetInertiaXX(chrono.ChVectorD(0.1,0.1,0.1))
    system.Add(load)

    joint_gear = chrono.ChLinkMateGeneric(True,True,True,False,True,True)
    joint_gear.Initialize(gear,ground,chrono.ChFrameD(chrono.ChVectorD(0,0,0)))
    system.AddLink(joint_gear)

    joint_damping = chrono.ChLinkRotSpringCB()
    joint_damping.Initialize(gear,ground,chrono.ChCoordsysD(chrono.ChVectorD(0,0,0)))
    joint_damping_torque = RotSpringTorque(0,b)
    joint_damping.RegisterTorqueFunctor(joint_damping_torque)
    system.AddLink(joint_damping)

    joint_load = chrono.ChLinkMateGeneric(True,True,True,False,True,True)
    joint_load.Initialize(load,gear,chrono.ChFrameD(load_pos))
    system.AddLink(joint_load)

    d['t_sim'] = []
    d['pos_sim'] = []

    def record():
        d['t_sim'].append(system.GetChTime())
        d['pos_sim'].append(np.arctan2(load.GetPos().z,load.GetPos().y))

    system.SetChTime(0)
    while system.GetChTime() <= tfinal:
        record()
        system.DoStepDynamics(step)

    return d
