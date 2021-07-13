import pychrono as chrono
import numpy as np

PI = np.pi
step_n = 2e-4 # Tune based on this time step

class Servo(chrono.ChFunction):
    def __init__(self, rot, pdi=[0,0,0], name=''):
        super().__init__()
        self.max_torque = 0.39*0.5 # percent of dynamixel XL-320 stall torque

        self.get_rot = None
        self.name = name

        self.t_d = None

        self.e_pre = 0
        self.e_sum = 0

        self.x_pre = 0
        self.out_pre = 0

        self.get_rot = rot

        self.p = pdi[0]
        self.d = pdi[1]
        self.i = pdi[2]

    def set_t(self, t):
        if t == self.t_d: return

        self.t_d = t
        self.e_pre = 0
        self.e_sum = 0

    def Get_y(self, x):
        if x == self.x_pre:
            out = self.out_pre
        elif self.t_d is None:
            out = 0
        else:
            step = x-self.x_pre

            t = self.get_rot()

            e = self.t_d - t
            if e < -PI: e = e + 2*PI
            if e > PI: e = e - 2*PI

            de = (e - self.e_pre)/step*step_n
            e_sum = self.e_sum + e*step/step_n

            out_pd = self.p*e + self.d*de
            out_i = self.i*e_sum
            out = out_pd + out_i

            # Integrator clamping
            if e*out > 0 and (out > self.max_torque or out < -self.max_torque):
                e_sum -= e
                out = out_pd



            # Limit max torque
            if out > self.max_torque:
                out = self.max_torque
            if out < -self.max_torque:
                out = -self.max_torque

            # if self.name == 'hip': print('t={:.3f} e={:.3f} de={:.3f} esum={:.3f} out={:.3f}'.format(x, e,de,e_sum, out))
            self.e_pre = e
            self.e_sum = e_sum

        self.x_pre = x
        self.out_pre = out

        return out
