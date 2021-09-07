import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'../')))

import matplotlib.pyplot as plt
import numpy as np

import leg.data as data

sim_data_files = [
    'data/multi_jump_sim_252.csv',
    'data/blind_multi_jump_sim_252_p25.csv'
]

t_range = 2.5

for sim_data_file in sim_data_files:
    sim_data = data.read(sim_data_file)
    ts = np.array(sim_data['time'])
    ys = np.array(sim_data['body_y'])

    ts_init_i = 0
    for i in range(len(ts)):
        if ts[i]>1:
            ts_init_i = i
            break
    ts_end_i = 0
    for i in range(len(ts)):
        if ts[i] > t_range+ts[ts_init_i]:
            ts_end_i = i
            break

    ts = ts[ts_init_i:ts_end_i]
    ys = ys[ts_init_i:ts_end_i]
    ts -= ts[0]
    ys -= ys[0]

    plt.plot(ts,ys)

plt.title('Simulated jump with optimized leg')
plt.xlabel('Time[s]')
plt.ylabel('Relative Vertical Movement[m]')
plt.show()
