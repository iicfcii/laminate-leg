import numpy as np
from scipy.optimize import differential_evolution, LinearConstraint

from model import Model
from leg import Leg
from controller import Jump
import sim

tf = 3
step = 2e-4

l = 0.1
l_min = 0.01
l_max = l

k_min = 100
k_max = 1000

lb = [0.03,0.08]

def toL(x):
    return [x[0],x[1],l-x[0]-x[1]]

def toK(x):
    return [x[2],x[3]]

def obj(x):
    try:
        leg = Leg(toL(x),toK(x),lb)
    except AssertionError:
        return 0

    model = Model(leg)
    controller = Jump(model)
    sim_data = sim.run(model, controller=controller, tfinal=tf, step=step, vis=False)
    h = average_height(sim_data)

    return -h

def height(sim_data):
    h_max = np.max(sim_data['body_y'])
    if np.abs(h_max) < 1e-4 or np.isnan(h_max): h_max = 0

    return h_max

def average_height(sim_data):
    hs = []
    for i in range(len(sim_data['body_dy'])-1):
        t = sim_data['time'][i]
        if t < 1: continue

        dy = sim_data['body_dy'][i]
        dy_next = sim_data['body_dy'][i+1]
        if dy*dy_next<0 and dy > 0 and t > 0.1:
            hs.append(sim_data['body_y'][i])

    return np.average(hs)

def total_energy(sim_data):
    hip_power = np.array(sim_data['hip_torque'])*np.array(sim_data['hip_speed'])
    crank1_power = np.array(sim_data['crank1_torque'])*np.array(sim_data['crank1_speed'])
    crank2_power = np.array(sim_data['crank2_torque'])*np.array(sim_data['crank2_speed'])

    hip_total_energy = np.sum(hip_power*step*(hip_power > 0))
    crank1_total_energy = np.sum(crank1_power*step*(crank1_power > 0))
    crank2_total_energy = np.sum(crank2_power*step*(crank2_power > 0))
    total_energy = hip_total_energy+crank1_total_energy+crank2_total_energy

    return total_energy

def cb(xk,convergence=0):
    print('l',toL(xk),'k',toK(xk),'convergence',convergence)

def run():
    res = differential_evolution(
        obj,
        bounds=[(l_min,l_max),(l_min,l_max),(k_min,k_max),(k_min,k_max)],
        constraints=LinearConstraint(np.array([[1,1,0,0]]),0,l),
        popsize=5,
        callback=cb,
        workers=-1,
        polish=False,
        disp=True
    )

    return res
