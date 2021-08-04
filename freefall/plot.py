import numpy as np
import matplotlib.pyplot as plt

import simulate

x = [0.00767821, 0.00068408]

plt.figure()
for m,c in zip([0.2,0.35,0.51,0.715],'rgbk'):
    d = simulate.run(*x,m=m)
    plt.plot(d['t_sim'], d['pos_sim'],c+'--')
    plt.plot(d['t_offset'], d['pos_offset'],c,label='m={:.2f}'.format(m))
plt.ylabel('Angle[rad]')
plt.xlabel('Time[s]')
plt.title('Simulation(--) vs Real(-) Servo Arm Free Fall')
plt.legend()
plt.show()
