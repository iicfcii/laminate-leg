import matplotlib.pyplot as plt

import optimize
import sim
import data
from model import Model
from leg import Leg
from controller import Jump

if __name__ ==  '__main__':
    res = optimize.run()
    print('Result', res.message)
    print('Config', res.x)
    print('Cost', res.fun)

    l = optimize.toL(res.x)
    k = optimize.toK(res.x)
    leg = Leg(l,k,optimize.lb)
    model = Model(leg)
    controller = Jump(model)
    sim_data = sim.run(model, controller=controller, tfinal=optimize.tf, step=optimize.step, vis=True)

    data.write(
        'data/opt_height.csv',
        ['l']+list(sim_data.keys()),
        [l]+list(sim_data.values())
    )

    plt.close('all')
    leg.plot(leg.q1, leg.q2)
    print('Height',optimize.average_height(sim_data))
