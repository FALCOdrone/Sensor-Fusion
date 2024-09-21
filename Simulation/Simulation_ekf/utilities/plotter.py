import matplotlib.pyplot as plt
import os
import math

# Read the data from imu.log
with open(os.path.join(os.path.dirname(__file__),'imu.log'), 'r') as file:
    data = file.readlines()

# Extract speed, and acceleration values
d = []

for line in data:
    d.append([float(value) for value in line.strip().split(',')])

# Create time axis
time = [[i*0.1 for i in range(len(d[j]))] for j in range(len(d))]

# Plot data
cols = 3
rows = math.ceil(len(d)/cols)

for graph in range(len(d)):
    plt.subplot(rows, cols, graph+1)
    plt.plot(time[graph], d[graph])
    #plt.xlabel('Time')
    #plt.title('Data ' + str(graph))

# Adjust layout and display the plot
#plt.tight_layout()
plt.show()