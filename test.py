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

lb = [0.03, 0.08]

# l = [0.039966029940051474, 0.04589268086621596, 0.014141289193732573]
# k = [999.3872170753998, 999.2124658221269]

# l = [0.01160912475345141, 0.0495495427586799, 0.038841332487868696]
# k = [2626.9241287484147, 1903.5400080474717]

l = [0.03,0.04,0.03]
k = [2000,2000]

leg = Leg(l,k,lb)

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

model = Model(leg, body_constraint='xyz')
controller = Jump(model)
sim_data = sim.run(model, controller=controller, tfinal=optimize.tf, step=optimize.step, vis=True, capture=0)
file_name = 'data/test.csv'
data.write(
    file_name,
    ['l']+['k']+list(sim_data.keys()),
    [l]+[k]+list(sim_data.values())
)
# sim_data = data.read(file_name)
# print('Height',optimize.average_height(sim_data))

plt.figure()
plt.plot(sim_data['time'], sim_data['hip_torque'])
plt.title('Torque')

plt.figure()
plt.plot(sim_data['time'], sim_data['hip_angle'])
plt.title('Angle')

plt.figure()
plt.plot(sim_data['time'], sim_data['leg_angle'])
plt.title('Leg')

plt.figure()
plt.plot(sim_data['time'], sim_data['body_rz'])
plt.title('Body')

plt.show()
