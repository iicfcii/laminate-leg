import numpy as np

import motor
from leg import Leg

PI = np.pi
DEG_2_RAD = PI/180

class Jump():
    t_settle = 0.5

    def __init__(self, model):
        self.model = model

        self.q1_i, self.q2_i = self.model.leg.est_ik(-PI/2,self.model.leg.lmax)

        self.balanced_counter = 0
        self.extended = False

        self.state = 'fixed'

        self.hip_servo = motor.Servo(
            lambda:self.model.link1.GetRot().Q_to_Euler123().z,
            pdi=[0.2,1.0,0.001],
            name='hip'
        )
        self.body_servo = motor.Servo(
            lambda:self.model.body.GetRot().Q_to_Euler123().z,
            pdi=[-0.2,-1,-0.0],
            name='body'
        )
        self.knee_servo = motor.Servo(
            lambda:self.model.motor_crank1.GetMotorRot(),
            pdi=[0.2,1.0,0.003],
            name='knee'
        )
        self.model.motor_crank1.SetTorqueFunction(self.knee_servo)

    def control(self):
        t = self.model.system.GetChTime()

        rz = self.model.body.GetRot().Q_to_Euler123().z
        drz = self.model.body.GetWvel_loc().z

        f_contact = self.model.link4.GetContactForce()
        f_contact = np.linalg.norm([f_contact.x,f_contact.y,f_contact.z])

        lr = 0.05
        le = 0.08

        if self.state == 'fixed':
            self.model.body.SetBodyFixed(True)
            self.model.motor_hip.SetTorqueFunction(self.hip_servo)
            q1, q2 = self.model.leg.est_ik(-PI/2,lr)
            self.hip_servo.set_t(q1)
            self.knee_servo.set_t(q2-self.q2_i)
            if t > Jump.t_settle:
                self.state = 'falling'

        if self.state == 'falling':
            self.model.body.SetBodyFixed(False)
            if f_contact > 0.01: self.state = 'balancing'

        if self.state == 'balancing':
            self.model.motor_hip.SetTorqueFunction(self.body_servo)
            self.body_servo.set_t(0)
            if np.abs(rz-0) < 0.05:
                self.balanced_counter += 1
            else:
                self.balanced_counter = 0

            if self.balanced_counter > 20:
                # print('balanced')
                self.state = 'extending'
                self.balanced_counter = 0

        if self.state == 'extending':
            self.model.motor_hip.SetTorqueFunction(self.hip_servo)
            q1, q2 = self.model.leg.est_ik(-PI/2,le)
            self.hip_servo.set_t(q1)
            self.knee_servo.set_t(q2-self.q2_i)
            if f_contact < 0.001:
                # print('extended')
                self.state = 'retracting'

        if self.state == 'retracting':
            self.model.motor_hip.SetTorqueFunction(self.hip_servo)
            q1, q2 = self.model.leg.est_ik(-PI/2,lr)
            self.hip_servo.set_t(q1)
            self.knee_servo.set_t(q2-self.q2_i)
            # print('retracted')
            self.state = 'falling'

class SetPoint():
    t_settle = 0.5

    def __init__(self, model):
        self.model = model

        self.q1_i, self.q2_i = self.model.leg.est_ik(-PI/2,self.model.leg.lmax)

        q1, q2 = self.model.leg.est_ik(-PI/2-PI/6,0.05)
        self.model.motor_hip_servo.set_t(q1-self.q1_i)
        self.model.motor_crank1_servo.set_t(q2-self.q2_i)

    def control(self):
        t = self.model.system.GetChTime()
