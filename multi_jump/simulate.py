import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'../')))

import matplotlib.pyplot as plt
import numpy as np

from model import Model
from leg import Leg
import controller
import sim
import optimize
import data

DEG_2_RAD = np.pi/180
PI = np.pi

plt.close('all')

# 30 deg
l = [0.02024755378975744, 0.04994252630525132, 0.029809919904991246]
k = [0.137646605366415, 0.18890156319206014]

# 90 deg
l = [0.020021439701442406, 0.05033526019793686, 0.02964330010062074]
k = [0.17275040453411444, 0.10060507993709034]

# l = [0.03,0.04,0.03]
# k = [0.2,0.2]

leg = Leg(l,k,optimize.lb)

# leg.plot(leg.q1,leg.q2)
# plt.title('Rest pose')
# leg.plot(*leg.ik_lookup[0,1:])
# plt.title('Retract pose')
# plt.figure()
# for l_ref, q1, q2 in leg.ik_lookup:
#     leg.plot(q1, q2, new=False)
# plt.title('Length workspace')
# plt.figure()
# for l_ref in np.linspace(leg.lmin,leg.lmax,30):
#     q1, q2 = leg.est_ik(-PI/2, l_ref)
#     leg.plot(q1, q2, new=False)
# plt.title('Length IK')
# plt.show()

model = Model(leg,dof='y')
controller = controller.MultiJump(model, contact_delay=0.02)
sim_data = sim.run(model, controller=controller, tfinal=optimize.tf, step=1e-4, vis=True, capture=0)
# file_name = 'data/multi_jump_sim_343.csv'
# data.write(
#     file_name,
#     ['l']+['k']+list(sim_data.keys()),
#     [l]+[k]+list(sim_data.values())
# )
# sim_data = data.read(file_name)
print('Height',optimize.average_height(sim_data))
print('Max deformation',optimize.max_deformation(sim_data))

plt.figure()
plt.subplot(211)
plt.plot(sim_data['time'], sim_data['body_x'])
plt.title('Body x')
plt.subplot(212)
plt.plot(sim_data['time'], sim_data['body_y'])
plt.title('Body y')

plt.figure()
plt.subplot(211)
plt.plot(sim_data['time'], sim_data['leg_length'])
plt.title('Leg length')
plt.subplot(212)
plt.plot(sim_data['time'], sim_data['leg_angle'])
plt.title('Leg angle')

plt.figure()
plt.plot(sim_data['time'], sim_data['contact_f'])
plt.title('Contact force')

plt.figure()
plt.plot(sim_data['time'], sim_data['spring1_deformation'], label='1')
plt.plot(sim_data['time'], sim_data['spring2_deformation'], label='2')
plt.title('Spring deformation')
plt.legend()

plt.show()
