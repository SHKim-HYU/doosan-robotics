#!/usr/bin/env python3
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np


# -------------------------------
# Plot the results
# -------------------------------

T_start = 0
T_end = sum(time_hist[k,1] - time_hist[k,0] for k in range(Nsim+1))

fig = plt.figure()

ax1 = plt.subplot(1,1,1)
ax1.plot(wp[0,:], wp[1,:], 'ko')
ax1.set_xlabel('moment_arm : py')
ax1.set_ylabel('moment_arm : pz')
ax1.set_xlim(-0.1,0.1)
ax1.set_ylim(-0.1,0.1)
ax1.set_aspect('equal', 'box')
