import matplotlib as mlb
import matplotlib.pyplot as plt
import numpy as np;
import pandas as pd; # Not needed unless using placeholder to read CSV

import  rclpy
from rclpy.node import Node
from rclpy.subscriber import Subscriber

# Grab assessed data
# - Assume it is numpy matrix, thus control & actual will be numpy array type
# - Top row = time
# - Bottom row = output parameters (degrees or revolution ticks)
# - Assumed to be all one row

# Pseudo-code for subscribing to a ROS topic and bulding




data = pd.read_csv('/Users/stephenwardle/Desktop/DataAnalysis/datasets/calibrate_data.csv') # Placeholder for testing
data = data.loc[:, ["Angle", "Tic"]]
data = data.to_numpy()
data_t = data.transpose()


# Initialise Plot Counters
plt_end = 0
lft_end = 0
m_tol = 0   # Gradient Tolerance

# Plot 1 - Left Turn
length = data.shape

print(data)

left_turn = np.empty((0,2), float)
for i in range(0, length[0]-1):
    left_turn = np.append(left_turn, [data[i]], axis=0)
    lft_end = lft_end + 1
    if (data[i+1][1] - data[i][1]) >= m_tol:
        break



right_turn = np.empty((0,2), float)
for i in range((lft_end), length[0]):
    right_turn = np.append(right_turn, [data[i]], axis=0)

left_turn_t = left_turn.transpose()
right_turn_t = right_turn.transpose()

print(left_turn_t)
print(right_turn_t)

left_fit = np.polyfit(left_turn_t[0], left_turn_t[1], deg=1)
right_fit = np.polyfit(right_turn_t[0], right_turn_t[1], deg=1)

pl = np.poly1d(left_fit)
pr = np.poly1d(right_fit)

pl_test = np.linspace(-90, 90, 100)
pr_test = np.linspace(-90, 90, 100)

print(left_fit)
print(right_fit)

fig, axs = plt.subplots(2)
fig.suptitle("Linear Regression of Turning Model")
fig.supxlabel("Steering Angle")
fig.supylabel("Encoder Ticks")
axs[0].scatter(left_turn_t[0], left_turn_t[1])
axs[0].plot(pl_test, pl(pl_test), '-')
axs[0].set_title("Left Turn Model")
axs[0].set_xlim([-5, 90])
axs[0].set_ylim([-8000, 0])

axs[1].scatter(right_turn_t[0], right_turn_t[1])
axs[1].plot(pr_test, pr(pr_test), '-')
axs[1].set_title("Right Turn Model")
axs[1].set_xlim([-90, 5])
axs[1].set_ylim([0, 8000])

plt.show()

# Add code here to send turning model polynomial