import matplotlib.pyplot as plt
import numpy as np

from model import Model
from leg import Leg
from controller import Jump, SimpleJump
import sim
import optimize
import data

DEG_2_RAD = np.pi/180
PI = np.pi

plt.close('all')

# l = [0.03,0.04,0.03]
# k = [500,500]

# k: 100-500
l = [0.04049039956352703, 0.028396032400047934, 0.031113568036425045]
k = [497.27456933056806, 471.0976364294928]

# k: 100-1000
l = [0.040260856392407424, 0.029048861969729906, 0.030690281637862676]
k = [975.0691632690518, 643.716130009568]

leg = Leg(l,k,optimize.lb)

leg.plot(leg.q1,leg.q2)
plt.title('Rest pose')
leg.plot(*leg.ik_lookup[0,1:])
plt.title('Retract pose')
# plt.figure()
# for l_ref, q1, q2 in leg.ik_lookup:
#     leg.plot(q1, q2, new=False)
# plt.title('Length workspace')
# plt.figure()
# for l_ref in np.linspace(lb[0],lb[1],30):
#     q1, q2 = leg.est_ik(-PI/2, l_ref)
#     leg.plot(q1, q2, new=False)
# plt.title('Length IK')

model = Model(leg,body_constraint='y')
controller = SimpleJump(model)
sim_data = sim.run(model, controller=controller, tfinal=optimize.tf, step=optimize.step, vis=True, capture=0)
file_name = 'data/test.csv'
data.write(
    file_name,
    ['l']+['k']+list(sim_data.keys()),
    [l]+[k]+list(sim_data.values())
)
# sim_data = data.read(file_name)
print('Height',optimize.average_height(sim_data))

plt.figure()
plt.plot(sim_data['time'], sim_data['hip_torque'])
plt.title('Torque')

plt.figure()
plt.subplot(211)
plt.plot(sim_data['time'], sim_data['body_x'])
plt.title('Body x')
plt.subplot(212)
plt.plot(sim_data['time'], sim_data['body_y'])
plt.title('Body y')

plt.figure()
plt.plot(sim_data['time'], sim_data['spring1_x'], label='spring 1')
plt.plot(sim_data['time'], sim_data['spring2_x'], label='spring 2')
plt.title('Spring x')
plt.legend()

plt.show()
