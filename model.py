import pychrono as chrono

import motor
from leg import Leg

# Single joint springs
class RotSpringTorque(chrono.TorqueFunctor):
    def __init__(self, k, b):
        super(RotSpringTorque, self).__init__()
        self.k = k
        self.b = b

    def __call__(self,time,angle,vel,link):
        torque = -self.k*angle-self.b*vel
        return torque

class Model:
    def __init__(self, leg, passive_ankle=True, body_constraint='y'):
        rho = leg.density()

        self.passive_ankle = passive_ankle
        self.leg = leg

        self.system = chrono.ChSystemNSC()
        self.system.Set_G_acc(chrono.ChVectorD(0,-9.81*1,0)) # No gravity

        chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.001)
        chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.001)

        contact_mat = chrono.ChMaterialSurfaceNSC()
        contact_mat.SetFriction(0.5)
        contact_mat.SetRestitution(0.3)

        # Bodies
        tg = 0.02
        ground = chrono.ChBodyEasyBox(0.5,tg,0.1,rho,True,True,contact_mat)
        ground.SetPos(chrono.ChVectorD(0,-tg/2,0))
        ground.SetRot(chrono.Q_from_AngZ(0))
        ground.SetBodyFixed(True)
        ground.GetCollisionModel().SetFamily(0)
        self.system.Add(ground)

        self.body = chrono.ChBodyEasyBox(*leg.link_dim(0),rho,True,True,contact_mat)
        self.body.SetPos(chrono.ChVectorD(*Leg.link_center(leg.link_pts(0))))
        self.body.SetRot(chrono.Q_from_AngZ(Leg.link_rotz(leg.link_pts(0))))
        self.body.GetCollisionModel().SetFamily(1)
        self.body.GetCollisionModel().SetFamilyMaskNoCollisionWithFamily(1)
        if body_constraint=='xyz': self.body.SetBodyFixed(True)
        self.system.Add(self.body)

        link1 = chrono.ChBodyEasyBox(*leg.link_dim(1),rho,True,True,contact_mat)
        link1.SetPos(chrono.ChVectorD(*Leg.link_center(leg.link_pts(1))))
        link1.SetRot(chrono.Q_from_AngZ(Leg.link_rotz(leg.link_pts(1))))
        link1.GetCollisionModel().SetFamily(1)
        link1.GetCollisionModel().SetFamilyMaskNoCollisionWithFamily(1)
        self.system.Add(link1)

        crank1 = chrono.ChBodyEasyBox(*leg.link_dim(4),rho,True,True,contact_mat)
        crank1.SetPos(chrono.ChVectorD(*Leg.link_center(leg.link_pts(4))))
        crank1.SetRot(chrono.Q_from_AngZ(Leg.link_rotz(leg.link_pts(4))))
        crank1.GetCollisionModel().SetFamily(1)
        crank1.GetCollisionModel().SetFamilyMaskNoCollisionWithFamily(1)
        self.system.Add(crank1)

        link2 = chrono.ChBodyEasyBox(*leg.link_dim(2),rho,True,True,contact_mat)
        link2.SetPos(chrono.ChVectorD(*Leg.link_center(leg.link_pts(2))))
        link2.SetRot(chrono.Q_from_AngZ(Leg.link_rotz(leg.link_pts(2))))
        link2.GetCollisionModel().SetFamily(1)
        link2.GetCollisionModel().SetFamilyMaskNoCollisionWithFamily(1)
        self.system.Add(link2)

        if not passive_ankle:
            crank2 = chrono.ChBodyEasyBox(*leg.link_dim(5),rho,True,True,contact_mat)
            crank2.SetPos(chrono.ChVectorD(*Leg.link_center(leg.link_pts(5))))
            crank2.SetRot(chrono.Q_from_AngZ(Leg.link_rotz(leg.link_pts(5))))
            crank2.GetCollisionModel().SetFamily(1)
            crank2.GetCollisionModel().SetFamilyMaskNoCollisionWithFamily(1)
            self.system.Add(crank2)

        link3 = chrono.ChBodyEasyBox(*leg.link_dim(3),rho,True,True,contact_mat)
        link3.SetPos(chrono.ChVectorD(*Leg.link_center(leg.link_pts(3))))
        link3.SetRot(chrono.Q_from_AngZ(Leg.link_rotz(leg.link_pts(3))))
        link3.GetCollisionModel().SetFamily(1)
        link3.GetCollisionModel().SetFamilyMaskNoCollisionWithFamily(1)
        # link3.SetBodyFixed(True)
        self.system.Add(link3)

        # Joints
        if body_constraint == 'y':
            joint_ground_body = chrono.ChLinkMateGeneric(True, False, True, True, True, True)
        else:
            joint_ground_body = chrono.ChLinkMateGeneric(False, False, True, True, True, False)
        joint_ground_body.Initialize(ground, self.body, chrono.ChFrameD(chrono.VNULL))
        self.system.Add(joint_ground_body)

        joint_body_link1 = chrono.ChLinkMateGeneric(True,True,True,True,True,False)
        joint_body_link1.Initialize(self.body,link1,chrono.ChFrameD(chrono.ChVectorD(*leg.link_pts(1)[:,0])))
        self.system.Add(joint_body_link1)

        joint_link1_crank1 = chrono.ChLinkMateGeneric(True,True,True,True,True,False)
        joint_link1_crank1.Initialize(link1,crank1,chrono.ChFrameD(chrono.ChVectorD(*leg.link_pts(4)[:,0])))
        self.system.Add(joint_link1_crank1)

        joint_link1_link2 = chrono.ChLinkMateGeneric(True,True,True,True,True,False)
        joint_link1_link2.Initialize(link1,link2,chrono.ChFrameD(chrono.ChVectorD(*leg.link_pts(1)[:,1])))
        self.system.Add(joint_link1_link2)

        if not passive_ankle:
            joint_link2_crank2 = chrono.ChLinkMateGeneric(True,True,True,True,True,False)
            joint_link2_crank2.Initialize(link2,crank2,chrono.ChFrameD(chrono.ChVectorD(*leg.link_pts(5)[:,0])))
            self.system.Add(joint_link2_crank2)

        joint_link2_link3 = chrono.ChLinkMateGeneric(True,True,True,True,True,False)
        joint_link2_link3.Initialize(link2,link3,chrono.ChFrameD(chrono.ChVectorD(*leg.link_pts(2)[:,1])))
        self.system.Add(joint_link2_link3)

        # Single joint springs
        spring_body_link1 = chrono.ChLinkRotSpringCB()
        spring_body_link1.Initialize(link1,self.body,chrono.ChCoordsysD(chrono.ChVectorD(*leg.link_pts(1)[:,0])))
        self.spring_body_link1_torque = RotSpringTorque(*leg.spring_kb(0))
        spring_body_link1.RegisterTorqueFunctor(self.spring_body_link1_torque)
        self.system.AddLink(spring_body_link1)

        spring_link1_link2 = chrono.ChLinkRotSpringCB()
        spring_link1_link2.Initialize(link2,link1,chrono.ChCoordsysD(chrono.ChVectorD(*leg.link_pts(1)[:,1])))
        self.spring_link1_link2_torque = RotSpringTorque(*leg.spring_kb(1))
        spring_link1_link2.RegisterTorqueFunctor(self.spring_link1_link2_torque)
        self.system.AddLink(spring_link1_link2)

        spring_link2_link3 = chrono.ChLinkRotSpringCB()
        spring_link2_link3.Initialize(link3,link2,chrono.ChCoordsysD(chrono.ChVectorD(*leg.link_pts(2)[:,1])))
        self.spring_link2_link3_torque = RotSpringTorque(*leg.spring_kb(2))
        spring_link2_link3.RegisterTorqueFunctor(self.spring_link2_link3_torque)
        self.system.AddLink(spring_link2_link3)

        # Double joint springs
        spring_crank1_link2 = chrono.ChLinkTSDA()
        spring_crank1_link2.SetSpringCoefficient(leg.spring_kb(3)[0])
        spring_crank1_link2.SetDampingCoefficient(leg.spring_kb(3)[1])
        spring_crank1_link2.Initialize(
            crank1,link2,False,
            chrono.ChVectorD(*leg.link_pts(4)[:,1]),chrono.ChVectorD(*leg.link_pts(2)[:,0]),
            True
        )
        self.system.AddLink(spring_crank1_link2)


        spring_crank2_link3 = chrono.ChLinkTSDA()
        spring_crank2_link3.SetSpringCoefficient(leg.spring_kb(4)[0])
        spring_crank2_link3.SetDampingCoefficient(leg.spring_kb(4)[1])
        spring_crank2_link3.Initialize(
            crank2 if not passive_ankle else link1,link3,False,
            chrono.ChVectorD(*leg.link_pts(5)[:,1]),chrono.ChVectorD(*leg.link_pts(3)[:,0]),
            True
        )
        self.system.AddLink(spring_crank2_link3)

        # Motors
        self.motor_hip = chrono.ChLinkMotorRotationTorque()
        self.motor_hip.Initialize(link1,self.body,chrono.ChFrameD(chrono.ChVectorD(*leg.link_pts(1)[:,0])))
        self.motor_hip_servo = motor.Servo(
            lambda:self.motor_hip.GetMotorRot(),
            pdi=[0.2,10,0.0005]
        )
        self.motor_hip.SetTorqueFunction(self.motor_hip_servo)
        self.system.Add(self.motor_hip)

        self.motor_crank1 = chrono.ChLinkMotorRotationTorque()
        self.motor_crank1.Initialize(crank1,link1,chrono.ChFrameD(chrono.ChVectorD(*leg.link_pts(4)[:,0])))
        self.motor_crank1_servo = motor.Servo(
            lambda:self.motor_crank1.GetMotorRot(),
            pdi=[0.2,1.0,0.0005]
        )
        self.motor_crank1.SetTorqueFunction(self.motor_crank1_servo)
        self.system.Add(self.motor_crank1)

        if not passive_ankle:
            self.motor_crank2 = chrono.ChLinkMotorRotationTorque()
            self.motor_crank2.Initialize(crank2,link2,chrono.ChFrameD(chrono.ChVectorD(*leg.link_pts(5)[:,0])))
            def motor_crank2_rot():
                return self.motor_crank2.GetMotorRot()
            self.motor_crank2_servo = motor.Servo(
                lambda:self.motor_crank2.GetMotorRot(),
                pdi=[0.2,0.1,0.0005]
            )
            self.motor_crank2.SetMotorFunction(self.motor_crank2_servo)
            self.system.Add(self.motor_crank2)

        # Visuals
        spring = chrono.ChPointPointSpring(0.001, 100, 20)
        color_red = chrono.ChColorAsset()
        color_red.SetColor(chrono.ChColor(1.0, 0, 0))
        color_green = chrono.ChColorAsset()
        color_green.SetColor(chrono.ChColor(0, 1.0, 0))

        crank1.AddAsset(color_red)
        if not passive_ankle: crank2.AddAsset(color_red)
        spring_crank1_link2.AddAsset(color_green)
        spring_crank1_link2.AddAsset(spring)
        spring_crank2_link3.AddAsset(color_green)
        spring_crank2_link3.AddAsset(spring)
