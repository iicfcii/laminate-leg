import matplotlib.pyplot as plt
import numpy as np

import data

t_range = 2.5

track_data = data.read('blind_multi_jump/jr_343_p2.csv')
t = track_data['t']
y = track_data['y']

y_init = np.average(y[:50])
t_init_i = 0
for i in range(len(y)):
    avg_y = np.average(y[i:i+10])
    if avg_y-y_init > 0.0005:
        t_init_i = i
        break

t_end_i = 0
for i in range(len(t)):
    if t[i] > t_range+t[t_init_i]:
        t_end_i = i
        break

t = np.array(t)[t_init_i:t_end_i]
y = np.array(y)[t_init_i:t_end_i]

t = t[y != np.array(None)]
y = y[y != np.array(None)]

t -= t[0]
y -= y[0]

sim_data = data.read('blind_multi_jump/sjr_343_p2.csv')
ts = sim_data['time']
ys = sim_data['body_y']

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

ts = np.array(ts)[ts_init_i:ts_end_i]
ts -= ts[0]
ys = np.array(ys)[ts_init_i:ts_end_i]
ys -= ys[0]

plt.figure()
plt.plot(t,y,label='exp')
plt.plot(ts,ys,label='sim')
plt.legend()
plt.show()
