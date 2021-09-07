import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'../')))

import matplotlib.pyplot as plt
import numpy as np

import leg.data as data

exp_data_files = [
    'data/blind_multi_jump_343_p20.csv',
    'data/blind_multi_jump_252_p23.csv'
]
sim_data_files = [
    'data/blind_multi_jump_sim_343_p22.csv',
    'data/blind_multi_jump_sim_252_p25.csv'
]
ratios = [
    '343',
    '252'
]
colors = [
    'r',
    'g'
]


t_range = 2.5

plt.figure()
for exp_data_file, sim_data_file, r, c in zip(exp_data_files,sim_data_files,ratios,colors):
    track_data = data.read(exp_data_file)
    t = np.array(track_data['t'])
    y = np.array(track_data['y'])

    # Clear None data
    t = t[y != np.array(None)]
    y = y[y != np.array(None)]

    # Find t starting and ending index
    y_init = np.average(y[:50])
    t_init_i = 0
    for i in range(len(y)):
        avg_y = np.average(y[i:i+10])
        if avg_y-y_init > 0.001:
            t_init_i = i
            break
    t_end_i = 0
    for i in range(len(t)):
        if t[i] > t_range+t[t_init_i]:
            t_end_i = i
            break

    # Trim and offset
    t = t[t_init_i:t_end_i]
    y = y[t_init_i:t_end_i]
    t -= t[0]
    y -= y[0]

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


    plt.plot(t,y,c+'-',label=r+' exp')
    plt.plot(ts,ys,c+'--',label=r+' sim')

plt.title('Blind jump of 2 different leg designs')
plt.xlabel('Time[s]')
plt.ylabel('Relative Vertical Movement[m]')
plt.legend()
plt.show()
