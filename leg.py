import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize, differential_evolution

import fourbar

DEG_2_RAD = np.pi/180
PI = np.pi

class Leg:
    # Link width and thickness
    w = 0.02
    t = 0.003
    h = 0.1

    def link_center(pts):
        return np.sum(pts,axis=1)/2

    def link_rotz(pts):
        return np.arctan2(pts[1,1]-pts[1,0],pts[0,1]-pts[0,0])

    def __init__(self, l1, l2, l3, k4, k5, q1=-60*DEG_2_RAD, q2=90*DEG_2_RAD):
        self.q1 = q1
        self.q2 = q2

        # Main leg length
        self.l0 = 0.05
        self.l0f = 0.025 # self.l0/2
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3

        # Double joint spring 1
        self.l4 = 0.02
        self.l5 = self.l1 # 0.04
        self.l6 = 0.015

        # Double joint spring 2
        self.l7 = 0.02
        self.l8 = self.l2 # 0.04
        self.l9 = 0.015

        # Solve fourbars
        # psf1, tsf1 = fourbar.calc(q2,self.l1,self.l4,l5,self.l6,x0=[0,0,0,self.l4,l5,self.l6,self.l1,0])
        psf1, tsf1 = fourbar.acalc(q2,self.l1,self.l4,self.l5,self.l6, form=0)
        q2p = tsf1[3]

        q3 = -(q2p+PI) # Align l7 to l1
        # psf2, tsf2 = fourbar.calc(q3,self.l2,self.l7,l8,self.l9,x0=[0,0,0,-self.l7,l8,-self.l9,self.l2,0])
        psf2, tsf2 = fourbar.acalc(q3,self.l2,self.l7,self.l8,self.l9, form=1)
        q3p = tsf2[3]

        # Transformations
        Tw1 = np.array([[np.cos(q1),-np.sin(q1),0,0],
                        [np.sin(q1),np.cos(q1),0,Leg.h],
                        [0,0,1,0],
                        [0,0,0,1]])
        T12 = np.array([[np.cos(q2p),-np.sin(q2p),0,self.l1],
                        [np.sin(q2p),np.cos(q2p),0,0],
                        [0,0,1,0],
                        [0,0,0,1]])
        T23 = np.array([[np.cos(q3p),-np.sin(q3p),0,self.l2],
                        [np.sin(q3p),np.cos(q3p),0,0],
                        [0,0,1,0],
                        [0,0,0,1]])
        T3t = np.array([[1,0,0,self.l3],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]])

        # Joint positions
        p_o = np.array([[0,0,0,1]]).T # joint origin
        self.p1 = Tw1@p_o
        self.p2 = Tw1@T12@p_o
        self.p3 = Tw1@T12@T23@p_o
        self.pt = Tw1@T12@T23@T3t@p_o

        # Body positions
        self.pbA = np.array([[self.l0f-self.l0,Leg.h,0,1]]).T
        self.pbB = np.array([[self.l0f,Leg.h,0,1]]).T

        # Fourbar positions
        self.ps1A = Tw1@np.array([[psf1[1,0],psf1[1,1],0,1]]).T
        self.ps1B = Tw1@np.array([[psf1[2,0],psf1[2,1],0,1]]).T

        self.ps2A = Tw1@T12@np.array([[psf2[1,0],psf2[1,1],0,1]]).T
        self.ps2B = Tw1@T12@np.array([[psf2[2,0],psf2[2,1],0,1]]).T

        self.k4 = k4
        self.k5 = k5

    def plot(self):
        # Plot
        plt.figure()

        pb = np.concatenate([self.pbA,self.pbB],axis=1)
        plt.plot(pb[0,:],pb[1,:],'-ok')

        pl = np.concatenate([self.p1,self.p2,self.p3,self.pt],axis=1)
        plt.plot(pl[0,:],pl[1,:],'-ok')

        ps1 = np.concatenate([self.p1,self.ps1A,self.ps1B,self.p2],axis=1)
        plt.plot(ps1[0,:],ps1[1,:],'--ok')

        ps2 = np.concatenate([self.p2,self.ps2A,self.ps2B,self.p3],axis=1)
        plt.plot(ps2[0,:],ps2[1,:],'--ok')

        plt.axis('scaled')
        plt.axis([-0.075,0.075,0,0.15])
        plt.title('Leg Configuration')

    def density(self):
        return 1000

    def spring_pts(self, n):
        ps = [
            np.concatenate([self.ps1A,self.ps1B],axis=1),
            np.concatenate([self.ps2A,self.ps2B],axis=1),
        ]
        return ps[n][0:3,:]

    def spring_kb(self, n):
        kbs = [
            [0.020, 0.0005],
            [0.020, 0.0005],
            [0.020, 0.0005],
            [self.k4, 0.1],
            [self.k5, 0.1]
        ]
        return kbs[n]

    def link_pts(self, n):
        ps = [
            np.concatenate([self.pbA,self.pbB],axis=1),
            np.concatenate([self.p1,self.p2],axis=1),
            np.concatenate([self.ps1B,self.p3],axis=1),
            np.concatenate([self.ps2B,self.pt],axis=1),
            np.concatenate([self.p1,self.ps1A],axis=1),
            np.concatenate([self.p2,self.ps2A],axis=1),
        ]
        return ps[n][0:3,:]

    def link_dim(self, n):
        ds = [
            [self.l0,Leg.t*10,Leg.w*2],
            [self.l1,Leg.t,Leg.w],
            [self.l2+self.l6,Leg.t,Leg.w],
            [self.l3+self.l9,Leg.t,Leg.w],
            [self.l4,Leg.t,Leg.w], # Crank1
            [self.l7,Leg.t,Leg.w], # Crank2
        ]
        return ds[n]

    def ik(self, t, l):
        def obj(x):
            ps_mid, ts_mid = fourbar.acalc(x[0],self.l2,self.l3,l,self.l1)
            if ts_mid is None: return 10
            ps_low, ts_low = fourbar.acalc(-ts_mid[-1],self.l2,self.l7,self.l8,self.l9)
            if ts_low is None: return 10

            xp = -(PI+ts_low[-1])

            return np.abs(x-xp)

        result = differential_evolution(obj,bounds=[(-PI,0),],popsize=10)
        assert result.fun < 1e-6
        x = result.x[0]
        ps_mid, ts_mid = fourbar.acalc(x,self.l2,self.l3,l,self.l1)
        ps_low, ts_low = fourbar.acalc(ts_mid[-1]+PI,self.l1,self.l6,self.l5,self.l4)
        q2 = -ts_low[-1]
        q1 = t+PI-ts_mid[2]+ts_mid[3]

        return q1, q2
