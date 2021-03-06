#!/usr/bin/env python
# Helper script to create a latched publisher for tf_static
import rospy
from tf2_msgs.msg import TFMessage

msg=TFMessage()

def callback(data):
    global msg
    if len(msg.transforms) == 0:
        msg = data
    else:
        msg.transforms = msg.transforms+ data.transforms
    rospy.loginfo("Received /tf_static_old and republishing latched /tf_static")
    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("tf_static_old", TFMessage, callback)
    pub = rospy.Publisher('tf_static', TFMessage, queue_size=10, latch=True)
    rospy.spin()
