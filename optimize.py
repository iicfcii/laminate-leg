import numpy as np
from scipy.optimize import differential_evolution, LinearConstraint

from model import Model
from leg import Leg
import controller
import sim
import data

tf = 4
step = 2e-4

l = 0.1
l_min = 0.02
l_max = l

k_min = 0.1
k_max = 1.0

d_max = 90/180*np.pi

lb = [0.05,0.09]

def toL(x):
    return [x[0],x[1],l-x[0]-x[1]]

def toK(x):
    return [x[-2],x[-1]]

def obj(x):
    try:
        leg = Leg(toL(x),toK(x),lb)
    except AssertionError:
        return 10

    model = Model(leg,dof='y')
    c = controller.DistMultiJump(model)
    sim_data = sim.run(model, controller=c, tfinal=tf, step=step, vis=False)

    if max_deformation(sim_data) > d_max:
        return 10

    h = average_height(sim_data)

    return -h

def max_deformation(sim_data):
    d = np.amax(np.abs(
        np.concatenate([
            np.array(sim_data['spring1_deformation']),
            np.array(sim_data['spring2_deformation'])
        ])
    ))
    return d

def height(sim_data):
    h_max = 0
    for i in range(len(sim_data['body_y'])-1):
        t = sim_data['time'][i]
        h = sim_data['body_y'][i]
        if t < controller.Jump.t_settle: continue

        if h > h_max:
            h_max = h

    return h_max

def average_height(sim_data):
    hs = []
    for i in range(len(sim_data['body_dy'])-1):
        t = sim_data['time'][i]
        if t < controller.MultiJump.t_settle: continue

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
        constraints=LinearConstraint(np.array([[1,1,0,0]]),0,l-l_min),
        popsize=5,
        callback=cb,
        workers=-1,
        polish=False,
        disp=True
    )
    print('Result', res.message)
    print('Config', res.x)
    print('Cost', res.fun)

    l_opt = toL(res.x)
    k_opt = toK(res.x)
    leg = Leg(l_opt,k_opt,lb)
    model = Model(leg,dof='y')
    c = controller.MultiJump(model)
    sim_data = sim.run(model, controller=c, tfinal=tf, step=step, vis=True)

    data.write(
        'data/opt_height.csv',
        ['l']+['k']+list(sim_data.keys()),
        [l_opt]+[k_opt]+list(sim_data.values())
    )

    leg.plot(leg.q1, leg.q2)
    print('Height',average_height(sim_data))

if __name__ == '__main__':
    run()
