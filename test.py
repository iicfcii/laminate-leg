import matplotlib.pyplot as plt
import numpy as np

from model import Model
from leg import Leg
from controller import SingleJump, MultipleJump, MultipleJumpBalanced
import sim
import optimize
import data

DEG_2_RAD = np.pi/180
PI = np.pi

plt.close('all')

# l = [0.045655514764815636, 0.043833482181127, 0.010511003054057368]
# k = [853.1554439648044, 777.2745390640325]

l = [0.03,0.04,0.03]
k = [500, 500]

# Rest pose
leg = Leg(*l,*k,q1=-60*DEG_2_RAD, q2=90*DEG_2_RAD)
leg.plot()

# IK
q1, q2 = leg.ik(-90*DEG_2_RAD,0.04)
leg = Leg(*l,*k,q1=q1,q2=q2)
leg.plot()

q1, q2 = leg.ik(-90*DEG_2_RAD,0.08)
leg = Leg(*l,*k,q1=q1,q2=q2)
leg.plot()

# model = Model(leg, body_constraint=None)
# controller = MultipleJump(model)

# sim_data = sim.run(model, controller=controller, tfinal=optimize.tf, step=optimize.step, vis=True, capture=0)
# file_name = 'data/test.csv'
# data.write(
#     file_name,
#     ['l']+list(sim_data.keys()),
#     [l]+list(sim_data.values())
# )
# sim_data = data.read(file_name)
# print('Height',optimize.average_height(sim_data))

# plt.figure()
# plt.plot(sim_data['time'], sim_data['hip_torque'], label='hip')
# plt.plot(sim_data['time'], sim_data['crank1_torque'], label='crank1')
# plt.title('Torque')
# plt.legend()
#
# plt.figure()
# plt.plot(sim_data['time'], sim_data['hip_angle'], label='hip')
# plt.plot(sim_data['time'], sim_data['crank1_angle'], label='crank1')
# plt.title('Angle')
# plt.legend()
#
# plt.figure()
# plt.subplot(311)
# plt.plot(sim_data['time'], sim_data['body_y'])
# plt.title('Body y,dy,ddy')
# plt.subplot(312)
# plt.plot(sim_data['time'], sim_data['body_dy'])
# plt.subplot(313)
# plt.plot(sim_data['time'], sim_data['body_ddy'])
#
# plt.figure()
# plt.plot(sim_data['time'], sim_data['body_rz'], label='rz')
# plt.title('Body rz')
# plt.legend()

plt.show()
