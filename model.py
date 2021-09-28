import pychrono as chrono
import numpy as np

import motor
from leg import Leg

PI = np.pi

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
    def __init__(self, leg, dof='xyz'):
        rho = leg.density()
        gear_m = 0.001
        gear_b = 0.00767821
        gear_I = 0.00068408

        self.leg = leg

        # default holder for all chrono objects
        # https://api.projectchrono.org/simulation_system.html#manual_ChSystem
        self.system = chrono.ChSystemNSC()
        # set gravity
        self.system.Set_G_acc(chrono.ChVectorD(0,-9.81*1,0))

        # set global contact parameters
        # https://api.projectchrono.org/collision_shapes.html#collision_models
        chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.001)
        chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.001)

        # https://api.projectchrono.org/collision_shapes.html#collision_materials
        contact_mat = chrono.ChMaterialSurfaceNSC()
        contact_mat.SetFriction(0.8)
        contact_mat.SetRestitution(0.3)

        # Bodies
        # https://api.projectchrono.org/rigid_bodies.html#manual_ChBody
        tg = 0.02 #thickness
        ground = chrono.ChBodyEasyBox(1.0,tg,0.1,rho,True,True,contact_mat)
        ground.SetPos(chrono.ChVectorD(0,-tg/2,0))
        ground.SetRot(chrono.Q_from_AngZ(0))
        ground.SetBodyFixed(True)
        # 
        ground.GetCollisionModel().SetFamily(0)
        self.system.Add(ground)

        self.body = chrono.ChBodyEasyBox(*leg.link_dim(0),rho,False,True,contact_mat)
        self.body.SetPos(chrono.ChVectorD(*Leg.link_center(leg.link_pts(0))))
        # self.body.SetPos(chrono.ChVectorD(
        #     Leg.link_center(leg.link_pts(0))[0],
        #     Leg.link_center(leg.link_pts(0))[1]-0.02,
        #     Leg.link_center(leg.link_pts(0))[2]
        # ))
        self.body.SetRot(chrono.Q_from_AngZ(0))
        # https://api.projectchrono.org/collision_shapes.html#collision_models
        self.body.GetCollisionModel().SetFamily(1)
        self.body.GetCollisionModel().SetFamilyMaskNoCollisionWithFamily(1) # up to 15 families
        self.system.Add(self.body)

        self.link1 = chrono.ChBodyEasyBox(*leg.link_dim(1),rho,True,True,contact_mat)
        self.link1.SetPos(chrono.ChVectorD(*Leg.link_center(leg.link_pts(1))))
        self.link1.SetRot(chrono.Q_from_AngZ(Leg.link_rotz(leg.link_pts(1))))
        self.link1.GetCollisionModel().SetFamily(1)
        self.link1.GetCollisionModel().SetFamilyMaskNoCollisionWithFamily(1)
        self.system.Add(self.link1)

        crank1 = chrono.ChBodyEasyBox(*leg.link_dim(4),rho,True,True,contact_mat)
        crank1.SetPos(chrono.ChVectorD(*Leg.link_center(leg.link_pts(4))))
        crank1.SetRot(chrono.Q_from_AngZ(Leg.link_rotz(leg.link_pts(4))))
        crank1.GetCollisionModel().SetFamily(1)
        crank1.GetCollisionModel().SetFamilyMaskNoCollisionWithFamily(1)
        self.system.Add(crank1)

        coupler1 = chrono.ChBodyEasyBox(*leg.link_dim(6),rho,True,True,contact_mat)
        coupler1.SetPos(chrono.ChVectorD(*Leg.link_center(leg.link_pts(6))))
        coupler1.SetRot(chrono.Q_from_AngZ(Leg.link_rotz(leg.link_pts(6))))
        coupler1.GetCollisionModel().SetFamily(1)
        coupler1.GetCollisionModel().SetFamilyMaskNoCollisionWithFamily(1)
        self.system.Add(coupler1)

        output1 = chrono.ChBodyEasyBox(*leg.link_dim(8),rho,True,True,contact_mat)
        output1.SetPos(chrono.ChVectorD(*Leg.link_center(leg.link_pts(8))))
        output1.SetRot(chrono.Q_from_AngZ(Leg.link_rotz(leg.link_pts(8))))
        output1.GetCollisionModel().SetFamily(1)
        output1.GetCollisionModel().SetFamilyMaskNoCollisionWithFamily(1)
        self.system.Add(output1)

        link2 = chrono.ChBodyEasyBox(*leg.link_dim(2),rho,True,True,contact_mat)
        link2.SetPos(chrono.ChVectorD(*Leg.link_center(leg.link_pts(2))))
        link2.SetRot(chrono.Q_from_AngZ(Leg.link_rotz(leg.link_pts(2))))
        link2.GetCollisionModel().SetFamily(1)
        link2.GetCollisionModel().SetFamilyMaskNoCollisionWithFamily(1)
        self.system.Add(link2)

        link3 = chrono.ChBodyEasyBox(*leg.link_dim(3),rho,True,True,contact_mat)
        link3.SetPos(chrono.ChVectorD(*Leg.link_center(leg.link_pts(3))))
        link3.SetRot(chrono.Q_from_AngZ(Leg.link_rotz(leg.link_pts(3))))
        link3.GetCollisionModel().SetFamily(1)
        link3.GetCollisionModel().SetFamilyMaskNoCollisionWithFamily(1)
        self.system.Add(link3)

        coupler2 = chrono.ChBodyEasyBox(*leg.link_dim(7),rho,True,True,contact_mat)
        coupler2.SetPos(chrono.ChVectorD(*Leg.link_center(leg.link_pts(7))))
        coupler2.SetRot(chrono.Q_from_AngZ(Leg.link_rotz(leg.link_pts(7))))
        coupler2.GetCollisionModel().SetFamily(1)
        coupler2.GetCollisionModel().SetFamilyMaskNoCollisionWithFamily(1)
        self.system.Add(coupler2)

        output2 = chrono.ChBodyEasyBox(*leg.link_dim(9),rho,True,True,contact_mat)
        output2.SetPos(chrono.ChVectorD(*Leg.link_center(leg.link_pts(9))))
        output2.SetRot(chrono.Q_from_AngZ(Leg.link_rotz(leg.link_pts(9))))
        output2.GetCollisionModel().SetFamily(1)
        output2.GetCollisionModel().SetFamilyMaskNoCollisionWithFamily(1)
        self.system.Add(output2)

        self.link4 = chrono.ChBodyEasySphere(0.005,rho,True,True,contact_mat)
        self.link4.SetPos(chrono.ChVectorD(*leg.link_pts(3)[:,1]))
        self.link4.SetRot(chrono.Q_from_AngZ(0))
        self.link4.GetCollisionModel().SetFamily(1)
        self.link4.GetCollisionModel().SetFamilyMaskNoCollisionWithFamily(1)
        self.system.Add(self.link4)

        # Joints
        if dof == 'y':
            joint_ground_body = chrono.ChLinkMateGeneric(True, False, True, True, True, True)
        elif dof == 'z':
            joint_ground_body = chrono.ChLinkMateGeneric(True, True, True, True, True, False)
        elif dof == 'xy':
            joint_ground_body = chrono.ChLinkMateGeneric(False, False, True, True, True, True)
        elif dof == 'yz':
            joint_ground_body = chrono.ChLinkMateGeneric(True, False, True, True, True, False)
        elif dof == 'xyz':
            joint_ground_body = chrono.ChLinkMateGeneric(False, False, True, True, True, False)
        else:
            joint_ground_body = chrono.ChLinkMateGeneric(True, True, True, True, True, True)
        joint_ground_body.Initialize(ground, self.body, chrono.ChFrameD(self.body.GetPos()))
        self.system.Add(joint_ground_body)

        # https://api.projectchrono.org/links.html

        joint_link1_link2 = chrono.ChLinkMateGeneric(True,True,True,True,True,False)
        joint_link1_link2.Initialize(self.link1,link2,chrono.ChFrameD(chrono.ChVectorD(*leg.link_pts(1)[:,1])))
        self.system.Add(joint_link1_link2)

        joint_link2_link3 = chrono.ChLinkMateGeneric(True,True,True,True,True,False)
        joint_link2_link3.Initialize(link2,link3,chrono.ChFrameD(chrono.ChVectorD(*leg.link_pts(2)[:,1])))
        self.system.Add(joint_link2_link3)

        joint_link3_link4 = chrono.ChLinkMateGeneric(True,True,True,True,True,True)
        joint_link3_link4.Initialize(link3,self.link4,chrono.ChFrameD(chrono.ChVectorD(*leg.link_pts(3)[:,1])))
        self.system.Add(joint_link3_link4)

        joint_crank1_coupler1 = chrono.ChLinkMateGeneric(True,True,True,True,True,False)
        joint_crank1_coupler1.Initialize(crank1,coupler1,chrono.ChFrameD(chrono.ChVectorD(*leg.link_pts(4)[:,1])))
        self.system.Add(joint_crank1_coupler1)

        joint_output1_coupler1 = chrono.ChLinkMateGeneric(True,True,True,True,True,False)
        joint_output1_coupler1.Initialize(output1,coupler1,chrono.ChFrameD(chrono.ChVectorD(*leg.link_pts(8)[:,0])))
        self.system.Add(joint_output1_coupler1)

        joint_output1_link2 = chrono.ChLinkMateGeneric(True,True,True,True,True,False)
        joint_output1_link2.Initialize(output1,link2,chrono.ChFrameD(chrono.ChVectorD(*leg.link_pts(8)[:,1])))
        self.system.Add(joint_output1_link2)

        joint_link1_coupler2 = chrono.ChLinkMateGeneric(True,True,True,True,True,False)
        joint_link1_coupler2.Initialize(self.link1,coupler2,chrono.ChFrameD(chrono.ChVectorD(*leg.link_pts(5)[:,1])))
        self.system.Add(joint_link1_coupler2)

        joint_output2_coupler2 = chrono.ChLinkMateGeneric(True,True,True,True,True,False)
        joint_output2_coupler2.Initialize(output2,coupler2,chrono.ChFrameD(chrono.ChVectorD(*leg.link_pts(9)[:,0])))
        self.system.Add(joint_output2_coupler2)

        joint_output2_link3 = chrono.ChLinkMateGeneric(True,True,True,True,True,False)
        joint_output2_link3.Initialize(output2,link3,chrono.ChFrameD(chrono.ChVectorD(*leg.link_pts(9)[:,1])))
        self.system.Add(joint_output2_link3)

        # Gearbox joints
        gear1 = chrono.ChBody()
        gear1.SetPos(chrono.ChVectorD(*leg.link_pts(1)[:,0]))
        gear1.SetRot(chrono.QUNIT)
        gear1.SetMass(gear_m)
        gear1.SetInertiaXX(chrono.ChVectorD(gear_I,gear_I,gear_I))
        self.system.Add(gear1)

        joint_gear1_link1 = chrono.ChLinkMateGeneric(True,True,True,True,True,True)
        joint_gear1_link1.Initialize(self.link1,gear1,chrono.ChFrameD(chrono.ChVectorD(*leg.link_pts(1)[:,0])))
        self.system.Add(joint_gear1_link1)

        damper_body_gear1 = chrono.ChLinkRotSpringCB()
        damper_body_gear1.Initialize(gear1,self.body,chrono.ChCoordsysD(chrono.ChVectorD(*leg.link_pts(1)[:,0])))
        self.damper_body_gear1_torque = RotSpringTorque(0,gear_b)
        damper_body_gear1.RegisterTorqueFunctor(self.damper_body_gear1_torque)
        self.system.AddLink(damper_body_gear1)

        gear2 = chrono.ChBody()
        gear2.SetPos(chrono.ChVectorD(*leg.link_pts(1)[:,0]))
        gear2.SetRot(chrono.QUNIT)
        gear2.SetMass(gear_m)
        gear2.SetInertiaXX(chrono.ChVectorD(gear_I,gear_I,gear_I))
        self.system.Add(gear2)

        joint_gear2_crank1 = chrono.ChLinkMateGeneric(True,True,True,True,True,True)
        joint_gear2_crank1.Initialize(crank1,gear2,chrono.ChFrameD(chrono.ChVectorD(*leg.link_pts(1)[:,0])))
        self.system.Add(joint_gear2_crank1)

        damper_body_gear2 = chrono.ChLinkRotSpringCB()
        damper_body_gear2.Initialize(gear2,self.body,chrono.ChCoordsysD(chrono.ChVectorD(*leg.link_pts(1)[:,0])))
        self.damper_body_gear2_torque = RotSpringTorque(0,gear_b)
        damper_body_gear2.RegisterTorqueFunctor(self.damper_body_gear2_torque)
        self.system.AddLink(damper_body_gear2)

        # Single joint springs
        spring_link1_crank1 = chrono.ChLinkRotSpringCB()
        spring_link1_crank1.Initialize(link2,self.link1,chrono.ChCoordsysD(chrono.ChVectorD(*leg.link_pts(1)[:,1])))
        self.spring_link1_crank1_torque = RotSpringTorque(*leg.spring_kb(0))
        spring_link1_crank1.RegisterTorqueFunctor(self.spring_link1_crank1_torque)
        self.system.AddLink(spring_link1_crank1)

        spring_link1_link2 = chrono.ChLinkRotSpringCB()
        spring_link1_link2.Initialize(link2,self.link1,chrono.ChCoordsysD(chrono.ChVectorD(*leg.link_pts(1)[:,1])))
        self.spring_link1_link2_torque = RotSpringTorque(*leg.spring_kb(1))
        spring_link1_link2.RegisterTorqueFunctor(self.spring_link1_link2_torque)
        self.system.AddLink(spring_link1_link2)

        spring_link2_link3 = chrono.ChLinkRotSpringCB()
        spring_link2_link3.Initialize(link3,link2,chrono.ChCoordsysD(chrono.ChVectorD(*leg.link_pts(2)[:,1])))
        self.spring_link2_link3_torque = RotSpringTorque(*leg.spring_kb(2))
        spring_link2_link3.RegisterTorqueFunctor(self.spring_link2_link3_torque)
        self.system.AddLink(spring_link2_link3)

        # Double joint springs
        # https://api.projectchrono.org/links.html
        # https://api.projectchrono.org/classchrono_1_1_ch_link_spring.html
        # https://api.projectchrono.org/classchrono_1_1_ch_link_t_s_d_a.html
        # https://api.projectchrono.org/classchrono_1_1_ch_link_rot_spring_c_b.html
        self.spring_output1_link2 = chrono.ChLinkRotSpringCB()
        self.spring_output1_link2.Initialize(output1,link2,chrono.ChCoordsysD(chrono.ChVectorD(*leg.link_pts(8)[:,1])))
        self.spring_output1_link2_torque = RotSpringTorque(*leg.spring_kb(3))
        self.spring_output1_link2.RegisterTorqueFunctor(self.spring_output1_link2_torque)
        self.system.AddLink(self.spring_output1_link2)

        self.spring_output2_link3 = chrono.ChLinkRotSpringCB()
        self.spring_output2_link3.Initialize(output2,link3,chrono.ChCoordsysD(chrono.ChVectorD(*leg.link_pts(9)[:,1])))
        self.spring_output2_link3_torque = RotSpringTorque(*leg.spring_kb(4))
        self.spring_output2_link3.RegisterTorqueFunctor(self.spring_output2_link3_torque)
        self.system.AddLink(self.spring_output2_link3)

        # Motors
        # https://api.projectchrono.org/motors.html
        self.motor_hip = chrono.ChLinkMotorRotationTorque()
        self.motor_hip.Initialize(self.link1,self.body,chrono.ChFrameD(chrono.ChVectorD(*leg.link_pts(1)[:,0])))
        self.system.Add(self.motor_hip)

        self.motor_crank1 = chrono.ChLinkMotorRotationTorque()
        self.motor_crank1.Initialize(crank1,self.link1,chrono.ChFrameD(chrono.ChVectorD(*leg.link_pts(4)[:,0])))
        self.system.Add(self.motor_crank1)

        # Visuals
        color_red = chrono.ChColorAsset()
        color_red.SetColor(chrono.ChColor(1.0, 0, 0))
        color_green = chrono.ChColorAsset()
        color_green.SetColor(chrono.ChColor(0, 1.0, 0))

        crank1.AddAsset(color_red)
