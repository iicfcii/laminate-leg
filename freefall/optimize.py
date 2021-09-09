import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import differential_evolution

import simulate

def obj(x):
    e_sum = 0

    for m in [0.35,0.51,0.715]:
        d = simulate.run(*x,m)
        pos = d['pos_sim']
        pos_offset = np.interp(d['t_sim'],d['t_offset'],d['pos_offset'])
        e_sum += np.sqrt(np.average((pos_offset-pos)**2))

    return e_sum

def cb(x,convergence=0):
    print('b',x[0],'I',x[1],'convergence',convergence)

res = differential_evolution(
    obj,
    bounds=[(1e-3,1e-1),(1e-4,1e-3)],
    popsize=5,
    callback=cb,
    workers=1,
    polish=False,
    disp=True
)

print('Result', res.message)
print('x', res.x)
print('Cost', res.fun)
