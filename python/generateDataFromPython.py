import numpy as np
import matplotlib.pyplot as plt
import control as ctrl

"""
This file generates test data for the floating-point pidtest
0. generate t array
1. generate controller input e(t)
2. generate controller output y(t)
3. write to file
"""

"""Parameters: 
filename: output file for writing data
e_mode: determines how the controller input e is generated
"""

filename = "PIDControlTestDataPython.txt"
filename_debug = "../build/debug/PIDControlTestData.txt"

# 0.5 and 30 according to given file
time_step = 0.5
t0 = 0
tf = 30

# e_mode = "sinus"
e_mode = "step"
step_time = 2
# e_mode = "two_added_sines"


"""Beginning of program execution"""

t = np.arange(t0, tf + time_step, time_step)

# generate different e(t)

if e_mode == "sinus":
    e = 2.5 * np.sin(t)

if e_mode == "step":
    e = np.heaviside(t-step_time, 0.5)

if e_mode == "two_added_sines":
    e = 1 * np.sin(t+np.pi/4) + 2 * np.sin(0.25*t+0)

# ################# initialize controllers

T_s = 0.5 # T_sample
K_p = 2
K_I = 0.5
K_D = 2
K_N = 0.5

# P-controller
# apparently the control toolbox cant simulate this simple system due to some strange bug

# PI-controller
pi_controller = ctrl.tf([K_p + K_I * T_s / 2, K_I * T_s / 2 - K_p], [1, -1], T_s)

# PID-controller
a1 = K_p + K_I * T_s / 2
a0 = K_I * T_s / 2 - K_p
num = [a1+K_D * K_N, -a1 + K_N * T_s * a1+a0-2*K_D*K_N, -a0 + K_N * T_s * a0 + K_D * K_N]
den = [1, K_N*T_s-2, -K_N * T_s + 1]
pid_controller = ctrl.tf(num, den, T_s)

print("PI-controller: " + str(pi_controller))
print("PID-controller: " + str(pid_controller))

# ################# simulate systems

t_pi, y_pi, x_pi = ctrl.forced_response(pi_controller, t, e)
t_pid, y_pid, x_pid = ctrl.forced_response(pid_controller, t, e)

if not (np.array_equal(t, t_pi) and np.array_equal(t, t_pid)):
    print("[FATAL ERROR] time arrays of pi/pid controller is not equal to t!")

# simulate p-controller
t_p = t
y_p = K_p * e

# write to file
X = np.stack((t, e, y_p, y_pi[0, :], y_pid[0, :]), axis=1)

np.savetxt(filename, X, "%.8f", delimiter="\t")
np.savetxt(filename_debug, X, "%.8f", delimiter="\t")

# plot results
fig = plt.figure(filename)
ax1 = fig.add_subplot(411)
ax1.plot(t, e, label="e")
ax1.set_xlabel("t")
ax1.set_ylabel("e")
ax1.grid()
ax1.legend()

ax2 = fig.add_subplot(412, sharex=ax1)
ax2.plot(t_pi, y_pi[0, :], label="PI controller output")
ax2.set_xlabel("t")
ax2.set_ylabel("PI controller output")
ax2.grid()
ax2.legend()

ax3 = fig.add_subplot(413, sharex=ax1)
ax3.plot(t_pid, y_pid[0, :], label="PID controller output")
ax3.set_xlabel("t")
ax3.set_ylabel("PID controller output")
ax3.grid()
ax3.legend()

ax4 = fig.add_subplot(414, sharex=ax1)
ax4.plot(t_p, y_p, label="P controller output")
ax4.set_xlabel("t")
ax4.set_ylabel("P controller output")
ax4.grid()
ax4.legend()

plt.show()



