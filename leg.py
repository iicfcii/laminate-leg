import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import differential_evolution

import fourbar

DEG_2_RAD = np.pi/180
PI = np.pi
np.warnings.filterwarnings('ignore', category=np.VisibleDeprecationWarning)

class Leg:
    w = 0.02 # Link dimension
    t = 0.002
    h = 0.1

    l0 = 0.05 # Body dimension
    l0f = 0.025
    w0 = 0.05
    t0 = 0.02

    t_ref = -PI/2 # Refence virtual leg angle

    def __init__(self, ls, ks, lb):
        # Leg links
        self.l1 = ls[0]
        self.l2 = ls[1]
        self.l3 = ls[2]

        # Double joint spring 1 links
        self.l4 = 0.02
        self.l5 = self.l1
        self.l6 = self.l4

        # Double joint spring 2 links
        self.l7 = 0.02
        self.l8 = self.l2
        self.l9 = self.l7

        self.k4 = ks[0]
        self.k5 = ks[1]

        self.lmin = lb[0]
        self.lmax = lb[1]

        self.ik_lookup = []
        for l in np.linspace(self.lmin,self.lmax,10):
            qs = self.ik(Leg.t_ref,l)
            assert qs is not None, 'l={:.3f} not reachable'.format(l)

            # Double check using fk
            # ik might find solution in the other form
            ps = self.fk(*qs)
            assert ps is not None, 'l={} ls={} qs={} is an incorrect four bar form'.format(l,[self.l1,self.l2,self.l3],qs)

            v_tip = ps[3][:,1] - np.array([0,Leg.h,0])
            l_fk = np.linalg.norm(v_tip)
            t_fk = np.arctan2(v_tip[1],v_tip[0])

            assert np.abs(l_fk-l) < 1e-5 and np.abs(t_fk-Leg.t_ref) < 1e-5, 'l={} ls={} qs={} is an incorrect four bar form'.format(l,[self.l1,self.l2,self.l3],qs)

            self.ik_lookup.append([l,*qs])
        self.ik_lookup = np.array(self.ik_lookup)

        self.q1 = self.ik_lookup[-1,1]  # Rest angle
        self.q2 = self.ik_lookup[-1,2]

    def fk(self,q1,q2):
        # Solve fourbars
        psf1, tsf1 = fourbar.calc(q2,self.l1,self.l4,self.l5,self.l6, form=0)
        if tsf1 is None: return None
        q2p = tsf1[3]

        q3 = -(q2p+PI) # Align l7 to l1
        psf2, tsf2 = fourbar.calc(q3,self.l2,self.l7,self.l8,self.l9, form=1)
        if tsf2 is None: return None
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
        p1 = Tw1@p_o
        p2 = Tw1@T12@p_o
        p3 = Tw1@T12@T23@p_o
        pt = Tw1@T12@T23@T3t@p_o

        # Body positions
        pbA = np.array([[Leg.l0f-Leg.l0,Leg.h,0,1]]).T
        pbB = np.array([[Leg.l0f,Leg.h,0,1]]).T

        # Fourbar positions
        ps1A = Tw1@np.array([[psf1[1,0],psf1[1,1],0,1]]).T
        ps1B = Tw1@np.array([[psf1[2,0],psf1[2,1],0,1]]).T
        
        ps2A = Tw1@T12@np.array([[psf2[1,0],psf2[1,1],0,1]]).T
        ps2B = Tw1@T12@np.array([[psf2[2,0],psf2[2,1],0,1]]).T

        ps = [
            np.concatenate([pbA,pbB],axis=1)[0:3,:], # Body
            np.concatenate([p1,p2],axis=1)[0:3,:], # link1
            np.concatenate([(ps1B+p2)/2,p3],axis=1)[0:3,:], # link2
            np.concatenate([(ps2B+p3)/2,pt],axis=1)[0:3,:], # link3
            np.concatenate([p1,ps1A],axis=1)[0:3,:], # Crank1
            np.concatenate([p2,ps2A],axis=1)[0:3,:], # Crank2
            np.concatenate([ps1A,ps1B],axis=1)[0:3,:], # Coupler1
            np.concatenate([ps2A,ps2B],axis=1)[0:3,:], # Coupler2
            np.concatenate([ps1B,(ps1B+p2)/2],axis=1)[0:3,:], # Output1
            np.concatenate([ps2B,(ps2B+p3)/2],axis=1)[0:3,:], # Output2
        ]

        return ps

    def plot(self, q1, q2, new=True):
        ps = self.fk(q1, q2)

        # Plot
        if new: plt.figure()
        plt.plot(ps[0][0,:],ps[0][1,:],'-ok')
        plt.plot(ps[1][0,:],ps[1][1,:],'-ok')
        plt.plot(ps[2][0,:],ps[2][1,:],'-ok')
        plt.plot(ps[3][0,:],ps[3][1,:],'-ok')
        plt.plot(ps[4][0,:],ps[4][1,:],'-or')
        plt.plot(ps[5][0,:],ps[5][1,:],'-or')
        plt.plot(ps[6][0,:],ps[6][1,:],'-ok')
        plt.plot(ps[7][0,:],ps[7][1,:],'-ok')

        plt.plot(ps[8][0,:],ps[8][1,:],'-og')
        plt.plot(ps[9][0,:],ps[9][1,:],'-og')

        plt.axis('scaled')
        plt.axis([-0.075,0.075,0,0.15])

    def ik(self, t, l):
        def obj(x):
            ps_mid, ts_mid = fourbar.calc(x[0],self.l2,self.l3,l,self.l1)
            if ts_mid is None: return 10
            ps_low, ts_low = fourbar.calc(-ts_mid[-1],self.l2,self.l7,self.l8,self.l9)
            if ts_low is None: return 10

            xp = -(PI+ts_low[-1])

            return np.abs(x-xp)

        trial_count = 0
        while True:
            result = differential_evolution(obj,bounds=[(-PI,0)],popsize=10)

            if result.fun < 1e-6:
                x = result.x[0]
                ps_mid, ts_mid = fourbar.calc(x,self.l2,self.l3,l,self.l1)
                ps_low, ts_low = fourbar.calc(ts_mid[-1]+PI,self.l1,self.l6,self.l5,self.l4)

                q2 = -ts_low[-1]
                q2p = -(PI+ts_mid[-1])
                q1 = t+PI-ts_mid[2]+ts_mid[3]

                if q2p > -PI and q2p < 0: return q1, q2 # Actual link2 angle is within -pi-0

            trial_count = trial_count + 1
            if trial_count > 9: return None

    def est_ik(self, t, l):
        lp = self.ik_lookup[:,0]
        q1p = self.ik_lookup[:,1]
        q2p = self.ik_lookup[:,2]

        q1 = np.interp(l, lp, q1p)
        q1 = t-Leg.t_ref+q1
        q2 = np.interp(l, lp, q2p)

        return q1, q2

    def link_center(pts):
        return np.sum(pts,axis=1)/2

    def link_rotz(pts):
        return np.arctan2(pts[1,1]-pts[1,0],pts[0,1]-pts[0,0])

    def limit_angle(t): # Limit angle range to -pi to pi
        t = np.fmod(t, 2*PI)
        if t > PI: t = t - 2*PI
        if t < -PI: t = t + 2*PI
        return t

    def density(self):
        return 1000

    def spring_kb(self, n):
        kbs = [
            [0.020, 0.0005],
            [0.020, 0.0005],
            [0.020, 0.0005],
            [self.k4, 0.0005],
            [self.k5, 0.0005]
        ]
        return kbs[n]

    def link_dim(self, n):
        ds = [
            [Leg.l0,Leg.t0,Leg.w0],
            [self.l1,Leg.t,Leg.w],
            [self.l2+self.l6/2,Leg.t,Leg.w],
            [self.l3+self.l9/2,Leg.t,Leg.w],
            [self.l4,Leg.t,Leg.w], # Crank1
            [self.l7,Leg.t,Leg.w], # Crank2
            [self.l5,Leg.t,Leg.w], # Coupler1
            [self.l8,Leg.t,Leg.w], # Coupler2
            [self.l6/2,Leg.t,Leg.w], # Output1
            [self.l9/2,Leg.t,Leg.w], # Output2
        ]
        return ds[n]

    def link_pts(self, n):
        ps = self.fk(self.q1, self.q2)
        return ps[n]
