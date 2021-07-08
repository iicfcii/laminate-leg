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

l = [0.042483213261197125, 0.04539294608707662, 0.01212384065172626]
k = [999.7529990102761, 980.1012318013156]

# l = [0.011385901998146312, 0.0470839607522986, 0.04153013724955509]
# k = [2964.6653072717927, 2862.6938048475604]

# l = [0.03,0.04,0.03]
# k = [1000,1000]

leg = Leg(l,k,lb)

leg.plot(leg.q1,leg.q2)
plt.title('Rest pose')
leg.plot(*leg.ik_lookup[0,1:])
plt.title('Retract pose')
plt.figure()
for l_ref, q1, q2 in leg.ik_lookup:
    leg.plot(q1, q2, new=False)
plt.title('Length workspace')
plt.figure()
for l_ref in np.linspace(lb[0],lb[1],30):
    q1, q2 = leg.est_ik(-PI/2, l_ref)
    leg.plot(q1, q2, new=False)
plt.title('Length IK')

model = Model(leg, body_constraint='y')
controller = Jump(model)
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
plt.plot(sim_data['time'], sim_data['hip_torque'], label='hip')
plt.plot(sim_data['time'], sim_data['crank1_torque'], label='crank1')
plt.title('Torque')
plt.legend()

plt.figure()
plt.plot(sim_data['time'], sim_data['hip_angle'], label='hip')
plt.plot(sim_data['time'], sim_data['crank1_angle'], label='crank1')
plt.title('Angle')
plt.legend()

plt.figure()
plt.subplot(311)
plt.plot(sim_data['time'], sim_data['body_y'])
plt.title('Body y,dy,ddy')
plt.subplot(312)
plt.plot(sim_data['time'], sim_data['body_dy'])
plt.subplot(313)
plt.plot(sim_data['time'], sim_data['body_ddy'])

# plt.figure()
# plt.plot(sim_data['time'], sim_data['body_rz'], label='rz')
# plt.title('Body rz')
# plt.legend()

plt.show()
