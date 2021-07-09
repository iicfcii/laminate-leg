import numpy as np

PI = np.pi
DEG_2_RAD = PI/180

class Jump():
    t_settle = 0.5
    tol_pose = 0.03
    threshold_pose = 100

    def __init__(self, model):
        self.model = model

        self.servo_offset = np.array(self.model.leg.est_ik(-PI/2,self.model.leg.lmax))
        self.angles_retracted = np.array(self.model.leg.est_ik(-PI/2,self.model.leg.lmin))-self.servo_offset
        self.angles_extended = np.array(self.model.leg.est_ik(-PI/2,self.model.leg.lmax))-self.servo_offset

        self.retracted_counter = 0
        self.extended_counter = 0
        self.ddy_pre = 0

        self.move(self.angles_retracted)

    def move(self,angles):
        self.model.motor_hip_servo.set_t(angles[0])
        self.model.motor_crank1_servo.set_t(angles[1])

    def control(self):
        t = self.model.system.GetChTime()

        ddy = self.model.body.GetPos_dtdt().y

        if t < Jump.t_settle:
            self.model.body.SetBodyFixed(True)
        else:
            self.model.body.SetBodyFixed(False)

        retracted = self.retracted_counter > Jump.threshold_pose
        extended = self.extended_counter > Jump.threshold_pose
        contact = (
            ddy*self.ddy_pre < 0 and
            self.ddy_pre < 0 and
            t > Jump.t_settle
        )

        if contact and retracted:
            # print('contact', t)
            self.move(self.angles_extended)
            self.retracted_counter = 0

        if extended:
            # print('extended', t)
            self.move(self.angles_retracted)
            self.extended_counter = 0

        # Leg pose detection
        if (
            np.abs(self.model.motor_hip.GetMotorRot()-self.angles_extended[0]) < Jump.tol_pose and
            np.abs(self.model.motor_crank1.GetMotorRot()-self.angles_extended[1]) < Jump.tol_pose
        ):
            self.extended_counter += 1
        else:
            self.extended_counter = 0

        if (
            np.abs(self.model.motor_hip.GetMotorRot()-self.angles_retracted[0]) < Jump.tol_pose and
            np.abs(self.model.motor_crank1.GetMotorRot()-self.angles_retracted[1]) < Jump.tol_pose
        ):
            self.retracted_counter += 1
        else:
            self.retracted_counter = 0

        self.ddy_pre = ddy
