from sensor_msgs.msg import PointCloud
import numpy as np
from ambf_client import Client
import rospy
from geometry_msgs.msg import Point32


client = Client()
client.connect()

sensor = client.get_obj_handle("/ambf/env/Prox")
rospy.sleep(2)
pub = rospy.Publisher('cloud', PointCloud, queue_size=10)
rate = rospy.Rate(1000)
theta = np.linspace(0, 2*np.pi, 20)

while 1:

    ranges = sensor.get_all_measurement()
    range = sensor.get_range(0)
    print(ranges)

    msg = PointCloud()
    msg.header.stamp = rospy.Time.now();
    msg.header.frame_id = "/stair"

    for r, angle in zip(ranges, theta):
        # if r == 0.0:
        #     r = 3.5
        point = Point32()
        print(r)
        point.x = (1.5-r) * np.cos(angle)
        point.y = (1.5-r) * np.sin(angle)
        point.z = 0.2
        msg.points.append(point)
    pub.publish(msg)

    rate.sleep()
