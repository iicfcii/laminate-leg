import numpy as np

import motor
from leg import Leg

PI = np.pi
DEG_2_RAD = PI/180

class Jump():
    t_settle = 0.5
    a_contact = 0.1

    def __init__(self, model):
        self.model = model

        self.q1_i, self.q2_i = self.model.leg.est_ik(-PI/2,self.model.leg.lmax)

        self.f_contact_pre = 0
        self.dx_pre = 0

        self.state = 'fixed'

        self.hip_servo = motor.Servo(
            lambda:self.model.link1.GetRot().Q_to_Euler123().z,
            pdi=[10,0,0.001],
            name='hip'
        )
        self.body_servo = motor.Servo(
            lambda:self.model.body.GetWvel_loc().z,
            pdi=[-0.2,-0.0,-0.001],
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

        x = self.model.body.GetPos().x
        dx = self.model.body.GetPos_dt().x
        rz = self.model.body.GetRot().Q_to_Euler123().z
        drz = self.model.body.GetWvel_loc().z

        f_contact = self.model.link4.GetContactForce()
        f_contact = np.linalg.norm([f_contact.x,f_contact.y,f_contact.z])
        f_contact = Jump.a_contact*f_contact+(1-Jump.a_contact)*self.f_contact_pre

        lr = 0.06
        le = 0.08
        lt = -PI/2-0.4*(0-x)-0.3*(0-dx)

        if self.state == 'fixed':
            self.model.body.SetBodyFixed(True)

            q1, q2 = self.model.leg.est_ik(lt,lr)
            self.model.motor_hip.SetTorqueFunction(self.hip_servo)
            self.hip_servo.set_t(q1)

            self.knee_servo.set_t(q2-self.q2_i)

            if t > Jump.t_settle:
                self.state = 'falling'

        if self.state == 'falling':
            self.model.body.SetBodyFixed(False)

            if f_contact > 1:
                # print(t, 'balancing')
                self.state = 'balancing'

        if self.state == 'balancing':
            self.model.motor_hip.SetTorqueFunction(self.body_servo)
            self.body_servo.set_t(None)
            self.body_servo.set_t(0)
            # print(t, 'extending')
            self.state = 'extending'

        if self.state == 'extending':
            q1, q2 = self.model.leg.est_ik(lt,le)
            self.knee_servo.set_t(q2-self.q2_i)

            if f_contact < 0.001:
                # print(t, 'retracting')
                self.state = 'retracting'

        if self.state == 'retracting':
            q1, q2 = self.model.leg.est_ik(lt,lr)

            self.model.motor_hip.SetTorqueFunction(self.hip_servo)
            self.hip_servo.set_t(None)
            self.hip_servo.set_t(q1)

            self.knee_servo.set_t(q2-self.q2_i)

            self.state = 'falling'

        self.f_contact_pre = f_contact
        self.dx_pre = dx

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
