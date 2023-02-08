import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# Read in the data
actual = pd.read_csv("src/control/steering/data/steering_reading_2023-02-01T05:55:34.csv")
desired = pd.read_csv("src/control/steering/data/driving_command_2023-02-01T05:55:34.csv")

# Plot the data on the same graph
# Convert time to seconds from start
actual["time"] = actual["time"] - actual["time"][0]
desired["time"] = desired["time"] - desired["time"][0]

plt.plot(actual["time"], actual["steering_angle"], label="Actual")
plt.plot(desired["time"], desired["drive.steering_angle"], label="Desired")
plt.legend()
plt.xlabel("Time (s)")
plt.ylabel("Steering Angle (deg)")

plt.xticks(np.arange(0, max(actual["time"]), 1))
plt.yticks(np.arange(0, max(max(desired["drive.steering_angle"]), max(actual["steering_angle"])), 2))

plt.grid(True)
plt.show()
