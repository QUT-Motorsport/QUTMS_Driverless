import matplotlib.pyplot as plt
import pandas as pd

# Read in the data
actual = pd.read_csv('data/steering_reading copy 7.csv')
desired = pd.read_csv('data/driving_command copy 7.csv')

# Plot the data on the same graph
# Convert time to seconds from start
actual['time'] = actual['time'] - actual['time'][0] - 0.3
desired['time'] = desired['time'] - desired['time'][0]

plt.plot(actual['time'], actual['steering_angle'], label='Actual')
plt.plot(desired['time'], desired['drive.steering_angle'], label='Desired')
plt.legend()
plt.xlabel('Time (s)')
plt.ylabel('Steering Angle (deg)')
plt.show()
