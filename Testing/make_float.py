import rospy
from ambf_client import Client


_client = Client()
_client.connect()
rate = rospy.Rate(1000)


handle = _client.get_obj_handle('Hip')

handle.set_pos(0,0,1)
while 1:
    rate.sleep()

