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

l = [0.020363460236168952, 0.05101704858511963, 0.02861949117871143]
k = [999.216631856817, 911.5225980715397]

# l = [0.03,0.04,0.03]
# k = [500, 500]

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

model = Model(leg,dof='y')
controller = controller.MultiJump(model)
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
plt.plot(sim_data['time'], sim_data['spring1_x'])
plt.plot(sim_data['time'], sim_data['spring2_x'])
plt.title('Spring deformation')

plt.show()
