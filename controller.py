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

        self.lr = 0.05
        self.le = 0.08
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
