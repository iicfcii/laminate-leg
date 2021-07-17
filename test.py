import matplotlib.pyplot as plt
import numpy as np

from model import Model
from leg import Leg
from controller import Jump
import sim
import optimize
import data

DEG_2_RAD = np.pi/180
PI = np.pi

plt.close('all')

# # max k 3000
# l = [0.04515207233110659, 0.04287188912111829, 0.011976038547775125]
# k = [2576.09682619896, 1978.0998986420916]

# # max k 2000
# l = [0.048032839599790045, 0.0385249302606569, 0.013442230139553063]
# k = [1370.406089914368, 1559.1034161966104]

# # max k 1000
# l = [0.05229882821338931, 0.03763365559476119, 0.010067516191849507]
# k = [993.6350437346102, 102.65713045659305]

l = [0.03,0.04,0.03]
k = [1000,1000]

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

model = Model(leg)
controller = Jump(model)
sim_data = sim.run(model, controller=controller, tfinal=10, step=optimize.step, vis=True, capture=0)
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
