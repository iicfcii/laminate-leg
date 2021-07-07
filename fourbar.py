import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

PI = np.pi
DEG_2_RAD = np.pi/180

def calc(bad,ad,ab,bc,cd,form=0):
    l = np.sqrt(ad**2+ab**2-2*ad*ab*np.cos(bad))

    cos_bcd = (bc**2+cd**2-l**2)/(2*bc*cd)
    if np.abs(cos_bcd) > 1 or np.isnan(cos_bcd): return None, None

    sin_bcd = np.sqrt(1-cos_bcd**2)
    if form == 1: sin_bcd = -sin_bcd

    cos_cdb = (l**2+cd**2-bc**2)/(2*l*cd)
    sin_cdb = bc/l*sin_bcd
    cdb = np.arctan2(sin_cdb,cos_cdb)
    adb = np.arcsin(ab/l*np.sin(bad))

    adc = adb+cdb
    # print(adc/DEG_2_RAD,adb/DEG_2_RAD,cdb/DEG_2_RAD)

    pts = np.array([
        [0,0],
        [ab*np.cos(bad),ab*np.sin(bad)],
        [cd*np.cos(PI-adc)+ad,cd*np.sin(PI-adc)],
        [ad,0]
    ])

    ts = np.zeros(4)
    for i in range(3):
        ts[i+1] = np.arctan2(pts[i+1,1]-pts[i,1],pts[i+1,0]-pts[i,0])

    return pts, ts

def calc_opt(crank_angle,ground_length,crank_length,coupler_length,output_length,x0,plot=False):
    def obj(pts,crank_angle,ground_length,crank_length,coupler_length,output_length):
        pts = pts.reshape((-1,2))
        vcrank = pts[1,:]-pts[0,:]
        vcoupler = pts[2,:]-pts[1,:]
        voutput = pts[2,:]-pts[3,:] # Flip direction so initial constraint value is around 0
        vground = pts[3,:]-pts[0,:]

        error = []

        # Length
        error.append(vcrank.dot(vcrank)-crank_length**2)
        error.append(vcoupler.dot(vcoupler)-coupler_length**2)
        error.append(voutput.dot(voutput)-output_length**2)
        error.append(vground.dot(vground)-ground_length**2)

        # Angle
        error.append(np.arctan2(vground[1],vground[0]))
        error.append(np.arctan2(vcrank[1],vcrank[0])-crank_angle) # Constraint crank angle can casuse the minimize to fail

        # Starting point
        error.append(pts[0,0])
        error.append(pts[0,1])

        error = np.array(error)

        return error.dot(error)


    result = minimize(obj,x0,args=(crank_angle,ground_length,crank_length,coupler_length,output_length))
    pts = result.x.reshape((-1,2))
    assert result.fun < 1e-6 # Make sure fourbar config is found correctly

    if plot: # Plot the initial pose
        pts_full = np.vstack([pts,pts[0,:]])
        plt.figure()
        plt.plot(pts_full[:,0],pts_full[:,1],'ko-')
        plt.axis('equal')

    ts = [0]
    for i in range(3):
        ts.append(np.arctan2(pts[i+1,1]-pts[i,1],pts[i+1,0]-pts[i,0]))
    ts =np.array(ts)

    return pts,ts
