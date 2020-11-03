from GaitAnaylsisToolkit.LearningTools.Runner import TPGMMRunner
import matplotlib.pyplot as plt


runner = TPGMMRunner.TPGMMRunner("/home/nathanielgoldfarb/catkin_ws/src/ambf_walker/config/walk2.pickle")

joints = []
count = 0
while count < runner.get_length():

    count += 1
    runner.step()
    x = runner.x
    joints.append(x[0])


print(joints)

plt.plot(joints)
plt.show()