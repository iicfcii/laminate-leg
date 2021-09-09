import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'../')))

import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import differential_evolution

import data
import motor

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

tfinal=1
step=2e-4

def run(b,I,deg):
    d = data.read('data/setpoint_{:d}.csv'.format(deg))

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

    # joint_gear = chrono.ChLinkMateGeneric(True,True,True,False,True,True)
    # joint_gear.Initialize(gear,ground,chrono.ChFrameD(chrono.ChVectorD(0,0,0)))
    # system.AddLink(joint_gear)

    joint_damping = chrono.ChLinkRotSpringCB()
    joint_damping.Initialize(gear,ground,chrono.ChCoordsysD(chrono.ChVectorD(0,0,0)))
    joint_damping_torque = RotSpringTorque(0,b)
    joint_damping.RegisterTorqueFunctor(joint_damping_torque)
    system.AddLink(joint_damping)

    servo = chrono.ChLinkMotorRotationTorque()
    servo.Initialize(gear,ground,chrono.ChFrameD(chrono.ChVectorD(0,0,0)))
    servo_torque = motor.Servo(
        lambda:servo.GetMotorRot(),
        pdi=[1,150,0],
    )
    servo_torque.set_t(deg*DEG_2_RAD)
    servo.SetTorqueFunction(servo_torque)
    system.Add(servo)

    d['t_sim'] = []
    d['pos_sim'] = []

    def record():
        d['t_sim'].append(system.GetChTime())
        d['pos_sim'].append(servo.GetMotorRot())

    system.SetChTime(0)
    while system.GetChTime() <= tfinal:
        record()
        system.DoStepDynamics(step)

    return d
