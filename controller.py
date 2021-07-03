import numpy as np

PI = np.pi
DEG_2_RAD = PI/180

class SingleJump():
    def __init__(self, model):
        self.model = model
        self.model.motor_hip_servo.set_t(0)
        self.model.motor_crank1_servo.set_t(0)

    def control(self):
        t = self.model.system.GetChTime()
        if t >= 0:
            self.model.motor_hip_servo.set_t(0)
            self.model.motor_crank1_servo.set_t(0)
        if t > 1.5:
            self.model.motor_hip_servo.set_t(PI/3)
            self.model.motor_crank1_servo.set_t(-PI/3)
        if t > 3:
            self.model.motor_hip_servo.set_t(0)
            self.model.motor_crank1_servo.set_t(0)

class MultipleJump():
    def __init__(self, model):
        self.model = model
        self.angles_retracted = [PI/3,-PI/3]
        self.angles_extended = [0,0]

        self.move(self.angles_extended)

        self.contact = False
        self.retracted = False
        self.ddy_pre = 0


    def move(self,angles):
        self.model.motor_hip_servo.set_t(angles[0])
        self.model.motor_crank1_servo.set_t(angles[1])

    def control(self):
        t = self.model.system.GetChTime()
        ddy = self.model.body.GetPos_dtdt().y
        tol = 0.05

        if self.contact:
            self.move(self.angles_extended)

        if not self.retracted:
            self.move(self.angles_retracted)

        # Contact detection
        if (
            ddy*self.ddy_pre < 0 and
            self.ddy_pre < 0 and
            self.retracted and
            not self.contact and
            t > 0.1
        ):
            # print('contact', t)
            self.contact = True

        # Leg pose detection
        if (
            np.abs(self.model.motor_hip.GetMotorRot()-self.angles_extended[0]) < tol and
            np.abs(self.model.motor_crank1.GetMotorRot()-self.angles_extended[1]) < tol and
            self.retracted
        ):
            # print('extend', t)
            self.retracted = False
            self.contact = False

        if (
            np.abs(self.model.motor_hip.GetMotorRot()-self.angles_retracted[0]) < tol and
            np.abs(self.model.motor_crank1.GetMotorRot()-self.angles_retracted[1]) < tol and
            not self.retracted
        ):
            # print('retract', t)
            self.retracted = True

        self.ddy_pre = ddy

class MultipleJumpBalanced():
    def __init__(self, model):
        self.model = model
        self.angles_retracted = [PI/3,-PI/3]
        self.angles_extended = [0,0]

        self.move(self.angles_extended)

        self.balancing = False
        self.contact = False
        self.retracted = False
        self.ddy_pre = 0



    def move(self,angles):
        self.model.motor_hip_servo.set_t(angles[0])
        self.model.motor_crank1_servo.set_t(angles[1])

    def control(self):
        t = self.model.system.GetChTime()
        ddy = self.model.body.GetPos_dtdt().y
        tol = 0.05

        if self.contact:
            self.model.motor_crank1_servo.set_t(0)
            # if not self.balancing:
            #     self.model.motor_hip_servo.setup(
            #         lambda:self.model.body.GetRot().Q_to_Euler123().z,
            #         pdi=[-0.2,-10,-0.0005]
            #     )
            #     self.model.motor_hip_servo.set_t(0)
            #     self.balancing = True
            # else:
            #     if np.abs(self.model.body.GetRot().Q_to_Euler123().z - 0) < tol:
            #         self.model.motor_hip_servo.setup(
            #             lambda:self.model.motor_hip.GetMotorRot(),
            #             pdi=[0.2,10,0.0005]
            #         )
            #         self.move(self.angles_extended)
        else:
            self.model.motor_crank1_servo.set_t(-PI/3)
            if not self.balancing:
                self.model.motor_hip_servo.setup(
                    lambda:self.model.body.GetRot().Q_to_Euler123().z,
                    pdi=[-0.2,-10,-0.0005]
                )
                self.model.motor_hip_servo.set_t(0)
                self.balancing = True
                print('balancing')


        # Contact detection
        if (
            ddy*self.ddy_pre < 0 and
            self.ddy_pre < 0 and
            t > 0.1 and
            not self.contact
        ):
            print('contact', t)
            self.contact = True

        # Leg pose detection
        if np.abs(self.model.motor_crank1.GetMotorRot()-0) < tol:
            print('extend', t)
            self.contact = False
            self.balancing = False
        #
        # if (
        #     np.abs(self.model.motor_hip.GetMotorRot()-self.angles_retracted[0]) < tol and
        #     np.abs(self.model.motor_crank1.GetMotorRot()-self.angles_retracted[1]) < tol and
        #     not self.retracted
        # ):
        #     print('retract', t)
        #     self.retracted = True


        self.ddy_pre = ddy
