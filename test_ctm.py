
from CTM import mpc_coordination
from CTM_transmission_only import celltransmission
import matplotlib.pyplot as plt

d0 = 3500   # demand road 1
d0_2 = 3400  # demand road 2
dr = 800  # Demand Ramp 1
rho1 = [10, 10, 10]  # density three cells road 1
rho2 = [10, 10, 10]  # density three cells road 2
q1 = 20
q2 = 20
w1 = 100
w2 = 100
input = [d0, d0_2, dr, rho1, rho2, q1, q2, w1, w2]

Ctrl_rm1 = []
Ctrl_rm3 = []
Ctrl_vsl1 = []
Ctrl_vsl2 = []
Ctrl_rg = []
#
# for i in range(5):
mpc_coordination(input)


dens1 = [[] for _ in range(3)]
dens2 = [[] for _ in range(3)]
speed1 = [[] for _ in range(3)]
speed2 = [[] for _ in range(3)]
flow1 = [[] for _ in range(3)]
flow2 = [[] for _ in range(3)]
[r1, r2, vsl1, vsl2, p] = [800, 800, 30, 30, 0.8]

# input = (d0, d0_2, dr, rho1, rho2, q1, q2, w1, w2, r1, r2, vsl1, vsl2, p)
# #input = [2460, 3480, 780, [26.666666666666668, 35.0, 23.333333333333336], [36.66666666666667, 30.0, 40.0], 0, 0, 0, 0, 800, 800, 30, 30, 0.8]
#
# horizon = 20
# for i in range(horizon):
#     input_1 = input[:9]
#     input_2 = input[9:14]
#     states = celltransmission(*input_1, *input_2)
#     input_1 = states[:9]
#     print(input_1)
#     density1 = states[3]
#     density2 = states[4]
#     speed_1 = states[14]
#     speed_2 = states[15]
#     for j in range(len(density1)):
#         dens1[j].append(density1[j])
#         dens2[j].append(density2[j])
#         speed1[j].append(speed_1[j])
#         speed2[j].append(speed_2[j])
#         flow1[j].append(density1[j]*speed_1[j])
#         flow2[j].append(density2[j] * speed_2[j])
#
#
# x = range(horizon)
# fid, ax = plt.subplots(2, 3)
# ax[0, 0].plot(x, dens1[0], label="density")
# ax[0, 0].plot(x, speed1[0], label="speed")
# ax[0, 1].plot(x, dens1[1], label="density")
# ax[0, 1].plot(x, speed1[1], label="speed")
# ax[0, 2].plot(x, dens1[2], label="density")
# ax[0, 2].plot(x, speed1[2], label="speed")
# ax[0, 2].legend()
# ax[1, 0].plot(x, dens2[0], label="density")
# ax[1, 0].plot(x, speed2[0], label="speed")
# ax[1, 1].plot(x, dens2[1], label="density")
# ax[1, 1].plot(x, speed2[1], label="speed")
# ax[1, 2].plot(x, dens2[2], label="density")
# ax[1, 2].plot(x, speed2[2], label="speed")
#
#
# fid, ax = plt.subplots(2, 3)
# ax[0, 0].plot(x, flow1[0], label="density")
# ax[0, 1].plot(x, flow1[1], label="density")
# ax[0, 2].plot(x, flow1[2], label="speed")
# ax[1, 0].plot(x, flow2[0], label="density")
# ax[1, 1].plot(x, flow2[1], label="speed")
# ax[1, 2].plot(x, flow2[2], label="density")
#


