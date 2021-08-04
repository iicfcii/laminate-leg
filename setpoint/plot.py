import numpy as np
import matplotlib.pyplot as plt

import simulate

x = [0.00767821, 0.00068408]

plt.figure()
for deg,c in zip([45,90,135,180],'rgbk'):
    d = simulate.run(*x,deg)
    plt.plot(d['t_sim'], d['pos_sim'],c+'--')
    plt.plot(d['t'], d['pos'],c,label='theta={:d}'.format(deg))
plt.ylabel('Angle[rad]')
plt.xlabel('Time[s]')
plt.title('Simulation(--) vs Real(-) Servo Setpoint')
plt.legend()
plt.show()
