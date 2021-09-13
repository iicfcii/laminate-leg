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
# l = [0.02140538223186566, 0.048980513420074936, 0.029614104348059403]
# k = [0.49417716238092957, 0.2918080377404415]

# 20 deg
# l = [0.021120768651613143, 0.04840123992774345, 0.03047799142064342]
# k = [0.5080892121371792, 0.4289675469821798]

# 30 deg, 0.3Nm
# l = [0.022756690892395952, 0.04796444618150568, 0.029278862926098367]
# k = [0.29658461762626237, 0.27781444982503745]

# 30 deg, 0.2Nm
l = [0.023993257950169322, 0.04769695021441758, 0.0283097918354131]
k = [0.19973607960316817, 0.19968553379164242]

# l = [0.03,0.04,0.03]
# k = [0.2,0.2]

leg = Leg(l,k,optimize.lb)

leg.plot(leg.q1,leg.q2)
plt.title('Rest pose')
plt.show()
# leg.plot(*leg.ik_lookup[0,1:])
# plt.title('Retract pose')
# plt.figure()
# for l_ref, q1, q2 in leg.ik_lookup:
#     leg.plot(q1, q2, new=False)
# plt.title('Length workspace')
# plt.figure()
# for l_ref in np.linspace(lb[0],lb[1],30):
#     q1, q2 = leg.est_ik(-PI/2, l_ref)
#     leg.plot(q1, q2, new=False)
# plt.title('Length IK')

model = Model(leg,dof='y')
controller = controller.MultiJump(model)
sim_data = sim.run(model, controller=controller, tfinal=5, step=optimize.step, vis=True, capture=0)
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

# plt.figure()
# plt.subplot(211)
# plt.plot(sim_data['time'], sim_data['leg_length'])
# plt.title('Leg length')
# plt.subplot(212)
# plt.plot(sim_data['time'], sim_data['leg_angle'])
# plt.title('Leg angle')
#
# plt.figure()
# plt.plot(sim_data['time'], sim_data['contact_f'])
# plt.title('Contact force')

plt.figure()
plt.plot(sim_data['time'], sim_data['spring1_deformation'], label='1')
plt.plot(sim_data['time'], sim_data['spring2_deformation'], label='2')
plt.title('Spring deformation')
plt.legend()

plt.show()
