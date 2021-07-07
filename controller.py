import numpy as np

PI = np.pi
DEG_2_RAD = PI/180

class Jump():
    def __init__(self, model):
        self.model = model

        servo_offset = np.array(self.model.leg.est_ik(-90*DEG_2_RAD,self.model.leg.lmax))

        self.angles_retracted = np.array(self.model.leg.est_ik(-90*DEG_2_RAD,self.model.leg.lmin))-servo_offset
        self.angles_extended = np.array(self.model.leg.est_ik(-90*DEG_2_RAD,self.model.leg.lmax))-servo_offset
        self.contact = True
        self.ddy_pre = 0

    def move(self,angles):
        self.model.motor_hip_servo.set_t(angles[0])
        self.model.motor_crank1_servo.set_t(angles[1])

    def control(self):
        t = self.model.system.GetChTime()

        if t < 1:
            self.move(self.angles_retracted)
            return

        ddy = self.model.body.GetPos_dtdt().y
        tol = 0.05

        if self.contact:
            self.move(self.angles_extended)
        else:
            self.move(self.angles_retracted)

        # Contact detection
        if (
            ddy*self.ddy_pre < 0 and
            self.ddy_pre < 0 and
            not self.contact
        ):
            # print('contact', t)
            self.contact = True

        if (
            np.abs(self.model.motor_hip.GetMotorRot()-self.angles_extended[0]) < tol and
            np.abs(self.model.motor_crank1.GetMotorRot()-self.angles_extended[1]) < tol and
            self.contact
        ):
            # print('No contact', t)
            self.contact = False

        self.ddy_pre = ddy
