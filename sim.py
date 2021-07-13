import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

from leg import Leg

chrono.SetChronoDataPath('C:/Users/cfc34/miniconda3/pkgs/pychrono-6.0.0-py37_223/Library/data/')
PI = np.pi

class Recorder:
    def __init__(self, model):
        self.model = model
        self.data = {
            'time': [],
            'hip_torque': [],
            'hip_angle': [],
            'leg_angle':[],
            'leg_length':[],
            'body_rz': []
        }

    def record(self):
        self.data['time'].append(self.model.system.GetChTime())

        self.data['hip_torque'].append(self.model.motor_hip.GetMotorTorque())
        self.data['hip_angle'].append(Leg.limit_angle(self.model.motor_hip.GetMotorRot()))

        self.data['body_rz'].append(self.model.body.GetRot().Q_to_Euler123().z)

        body_pos = self.model.body.GetPos()
        body_pos = np.array([body_pos.x,body_pos.y,body_pos.z])
        tip_pos = self.model.link4.GetPos()
        tip_pos = np.array([tip_pos.x,tip_pos.y,tip_pos.z])
        leg_v = tip_pos-body_pos
        leg_length = np.linalg.norm(leg_v)
        leg_angle = np.arctan2(leg_v[1],leg_v[0])

        self.data['leg_angle'].append(leg_angle)
        self.data['leg_length'].append(leg_length)

def run(model, controller=None, tfinal=5, step=5e-4, vis=True, capture=0):
    system = model.system
    recorder = Recorder(model)

    if vis:
        application = chronoirr.ChIrrApp(system, "Leg", chronoirr.dimension2du(1024, 768),chronoirr.VerticalDir_Y)
        application.AddTypicalSky()
        application.AddTypicalLights()
        y_offset = 0.1
        z_offset = -0.2
        application.AddTypicalCamera(chronoirr.vector3df(0, y_offset, z_offset),chronoirr.vector3df(0, y_offset, 0))
        application.AssetBindAll()
        application.AssetUpdateAll()
        if capture>0:
            application.SetVideoframeSaveInterval(int(1/step/capture))
            application.SetVideoframeSave(True)

        application.SetTimestep(step)
        # application.SetTryRealtime(True)

        while application.GetDevice().run():
            recorder.record()
            if controller is not None: controller.control()

            application.BeginScene()
            application.DrawAll()

            # Draw axis for scale and orientation
            l = 0.1
            chronoirr.drawSegment(application.GetVideoDriver(),chrono.ChVectorD(0,0,0),chrono.ChVectorD(l,0,0),chronoirr.SColor(1,255,0,0))
            chronoirr.drawSegment(application.GetVideoDriver(),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,l,0),chronoirr.SColor(1,0,255,0))
            chronoirr.drawSegment(application.GetVideoDriver(),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,l),chronoirr.SColor(1,0,0,255))

            application.DoStep()
            application.EndScene()

            # print('t: {:.3f}'.format(system.GetChTime()))
            if system.GetChTime() > tfinal: # in system seconds
                  application.GetDevice().closeDevice()
    else:
        system.SetChTime(0)
        while system.GetChTime() <= tfinal:
            recorder.record()
            if controller is not None: controller.control()
            system.DoStepDynamics(step)

    return recorder.data
