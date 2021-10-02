import numpy as np

import motor
from leg import Leg

PI = np.pi
DEG_2_RAD = PI/180

class Jump():
    t_settle = 1.0
    a_contact = 0.1

    def __init__(self, model):
        self.model = model

        self.q1_i, self.q2_i = self.model.leg.est_ik(-PI/2,self.model.leg.lmax)

        self.lr = self.model.leg.lmin
        self.le = self.model.leg.lmax
        self.lt = -PI/2

        self.state = 'falling'

        self.hip_servo = motor.Servo(
            lambda:self.model.motor_hip.GetMotorRot(),
            pdi=[1,150,0],
            name='hip'
        )
        self.knee_servo = motor.Servo(
            lambda:self.model.motor_crank1.GetMotorRot(),
            pdi=[1,150,0],
            name='knee'
        )
        self.model.motor_crank1.SetTorqueFunction(self.knee_servo)
        self.model.motor_hip.SetTorqueFunction(self.hip_servo)

    def control(self):
        t = self.model.system.GetChTime()

        if self.state == 'falling':
            self.model.body.SetBodyFixed(False)
            q1, q2 = self.model.leg.est_ik(self.lt,self.lr)
            self.hip_servo.set_t(q1-self.q1_i)
            self.knee_servo.set_t(q2-self.q2_i)

            if t > Jump.t_settle:
                self.state = 'extending'

        if self.state == 'extending':
            q1, q2 = self.model.leg.est_ik(self.lt,self.le)
            self.hip_servo.set_t(q1-self.q1_i)
            self.knee_servo.set_t(q2-self.q2_i)

class MultiJump():
    t_settle = 1.0
    a_contact = 0.1

    def __init__(self, model, contact_delay=None):
        self.model = model

        self.q1_i, self.q2_i = self.model.leg.est_ik(-PI/2,self.model.leg.lmax)

        self.lr = self.model.leg.lmin
        self.le = self.model.leg.lmax
        self.lt = -PI/2

        self.f_contact_pre = 0
        self.state = 'falling'

        self.hip_servo = motor.Servo(
            lambda:self.model.motor_hip.GetMotorRot(),
            pdi=[1,150,0],
            name='hip'
        )
        self.knee_servo = motor.Servo(
            lambda:self.model.motor_crank1.GetMotorRot(),
            pdi=[1,150,0],
            name='knee'
        )
        self.model.motor_crank1.SetTorqueFunction(self.knee_servo)
        self.model.motor_hip.SetTorqueFunction(self.hip_servo)

        self.t_contact = None
        self.contact_delay = contact_delay

    def control(self):
        t = self.model.system.GetChTime()

        f_contact = self.model.link4.GetContactForce()
        f_contact = np.linalg.norm([f_contact.x,f_contact.y,f_contact.z])
        f_contact = Jump.a_contact*f_contact+(1-Jump.a_contact)*self.f_contact_pre

        if self.state == 'falling':
            self.model.body.SetBodyFixed(False)
            q1, q2 = self.model.leg.est_ik(self.lt,self.lr)
            self.hip_servo.set_t(q1-self.q1_i)
            self.knee_servo.set_t(q2-self.q2_i)

            if t > Jump.t_settle:
                self.state = 'extending'

        if self.state == 'extending':
            q1, q2 = self.model.leg.est_ik(self.lt,self.le)
            self.hip_servo.set_t(q1-self.q1_i)
            self.knee_servo.set_t(q2-self.q2_i)

            if f_contact < 0.01:
                self.state = 'retracting'

        if self.state == 'retracting':
            q1, q2 = self.model.leg.est_ik(self.lt,self.lr)
            self.hip_servo.set_t(q1-self.q1_i)
            self.knee_servo.set_t(q2-self.q2_i)

            if f_contact > 1:
                self.state = 'delay'

        if self.state == 'delay':
            if self.contact_delay is None:
                self.state = 'extending'
            else:
                if self.t_contact is None:
                    self.t_contact = t
                else:
                    if t-self.t_contact > self.contact_delay:
                        self.t_contact = None
                        self.state = 'extending'

        self.f_contact_pre = f_contact

class BlindMultiJump():
    t_settle = 1.0

    def __init__(self, model, period):
        self.model = model

        self.q1_i, self.q2_i = self.model.leg.est_ik(-PI/2,self.model.leg.lmax)

        self.lr = self.model.leg.lmin
        self.le = self.model.leg.lmax
        self.lt = -PI/2
        self.p = period

        self.state = 'falling'
        self.t_pre = None

        self.hip_servo = motor.Servo(
            lambda:self.model.motor_hip.GetMotorRot(),
            pdi=[1,150,0],
            name='hip'
        )
        self.knee_servo = motor.Servo(
            lambda:self.model.motor_crank1.GetMotorRot(),
            pdi=[1,150,0],
            name='knee'
        )
        self.model.motor_crank1.SetTorqueFunction(self.knee_servo)
        self.model.motor_hip.SetTorqueFunction(self.hip_servo)

    def control(self):
        t = self.model.system.GetChTime()

        if self.state == 'falling':
            self.model.body.SetBodyFixed(False)
            q1, q2 = self.model.leg.est_ik(self.lt,self.lr)
            self.hip_servo.set_t(q1-self.q1_i)
            self.knee_servo.set_t(q2-self.q2_i)

            if t > Jump.t_settle:
                self.state = 'extending'

        if self.state == 'extending':
            q1, q2 = self.model.leg.est_ik(self.lt,self.le)
            self.hip_servo.set_t(q1-self.q1_i)
            self.knee_servo.set_t(q2-self.q2_i)

            if self.t_pre is None:
                self.t_pre = t
            else:
                if t - self.t_pre > self.p/2:
                    self.state = 'retracting'
                    self.t_pre = None

        if self.state == 'retracting':
            q1, q2 = self.model.leg.est_ik(self.lt,self.lr)
            self.hip_servo.set_t(q1-self.q1_i)
            self.knee_servo.set_t(q2-self.q2_i)

            if self.t_pre is None:
                self.t_pre = t
            else:
                if t - self.t_pre > self.p/2:
                    self.state = 'extending'
                    self.t_pre = None
