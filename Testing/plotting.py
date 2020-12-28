import matplotlib.pyplot as plt
from matplotlib.pyplot import Figure
data0 = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
data1 = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
data2 = [9, 8, 7, 6, 5, 4, 3, 2, 1, 0]
data3 = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
data4 = [9, 8, 7, 6, 5, 4, 3, 2, 1, 0]
data5 = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
data6 = [9, 8, 7, 6, 5, 4, 3, 2, 1, 0]
data7 = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
data8 = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
data9 = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]

joint_fig = plt.figure()
joint_fig.suptitle('Walking Gait Trajectory Control', fontsize=20, verticalalignment='top')
tau_fig = plt.figure()
tau_fig.suptitle("Tau Effort of Walking Gait", fontsize=20, verticalalignment='center')

ax1 = joint_fig.add_subplot(311)
ax1.plot(data0, data1, label='Actual')
ax1.plot(data0, data2, label='Trajectory')
ax1.legend()
ax1.set_title("Hip", fontsize=15, verticalalignment='top')
ax1.set_ylabel("Joint Position \n (Degrees)", fontsize=12, verticalalignment="bottom")
ax1.get_xaxis().set_visible(False)

ax2 = joint_fig.add_subplot(312)
ax2.plot(data0, data3, label='Actual')
ax2.plot(data0, data4, label='Trajectory')
ax2.legend()
ax2.set_title("Knee", fontsize=15, verticalalignment='top')
ax2.set_ylabel("Joint Position \n (Degrees)", fontsize=12, verticalalignment="bottom")
ax2.get_xaxis().set_visible(False)

ax3 = joint_fig.add_subplot(313)
ax3.plot(data0, data5, label='Actual')
ax3.plot(data0, data6, label='Trajectory')
ax3.legend()
ax3.set_title("Ankle", fontsize=15, verticalalignment='top')
ax3.set_ylabel("Joint Position \n (Degrees)", fontsize=12, verticalalignment="bottom")
ax3.set_xlabel("Walking Gait Percentage", fontsize=12, verticalalignment="top")

ax4 = tau_fig.add_subplot(311)
ax4.plot(data0, data7)
ax4.set_title('Torque Output RH', fontsize=20, verticalalignment='top')

ax5 = tau_fig.add_subplot(312)
ax5.plot(data0, data8)
ax5.set_title('Torque Output RK')

ax6 = tau_fig.add_subplot(313)
ax6.plot(data0, data9)
ax6.set_title('Torque Output RA')

plt.show()
