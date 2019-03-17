import numpy as np
import matplotlib.pyplot as plt

filename_python = "../PIDControlTestDataPython.txt"
filename_matlab = "PIDControlTestDataMatlab.txt"

data_python = np.loadtxt(filename_python)
data_matlab = np.loadtxt(filename_matlab)

if np.array_equal(data_python, data_matlab):
    print("Both data files are exactly equal!")

fig = plt.figure("Comparision of Matlab and Python test data")
fig.suptitle("Comparision of Matlab and Python test data")

ax1 = fig.add_subplot(411)
ax1.plot(data_python[:, 0], data_python[:, 1], label="e/python")
ax1.plot(data_matlab[:, 0], data_matlab[:, 1], label="e/matlab")
ax1.set_xlabel("t")
ax1.set_ylabel("e")
ax1.legend()

ax2 = fig.add_subplot(412, sharex=ax1)
ax2.plot(data_python[:, 0], data_python[:, 2], label="P-Controller/python")
ax2.plot(data_matlab[:, 0], data_matlab[:, 2], label="P-Controller/matlab")
ax2.set_xlabel("t")
ax2.set_ylabel("P-Controller")
ax2.legend()

ax3 = fig.add_subplot(413, sharex=ax1)
ax3.plot(data_python[:, 0], data_python[:, 3], label="PI-Controller/python")
ax3.plot(data_matlab[:, 0], data_matlab[:, 3], label="PI-Controller/matlab")
ax3.set_xlabel("t")
ax3.set_ylabel("PI-Controller")
ax3.legend()

ax4 = fig.add_subplot(414, sharex=ax1)
ax4.plot(data_python[:, 0], data_python[:, 4], label="PID-Controller/python")
ax4.plot(data_matlab[:, 0], data_matlab[:, 4], label="PID-Controller/matlab")
ax4.set_xlabel("t")
ax4.set_ylabel("PID-Controller")
ax4.legend()

plt.show()
