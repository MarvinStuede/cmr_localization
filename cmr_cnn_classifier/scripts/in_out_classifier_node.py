#!/usr/bin/env python
##@package in_out_classifier_node
#Implements a node for indoor/outdoor classification based on ROS Image message

import sys, time
import numpy as np
from scipy.ndimage import filters
import cv2
import roslib
import rospy
from in_out_classifier import in_out_classifier
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, Image
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from cv_bridge import CvBridge, CvBridgeError

##in_out_classifier_node class.
class in_out_classifier_node:

    ##The constructor
    def __init__(self):

        # Parameters
        self.img_transport = rospy.get_param('~img_transport','compressed')
        self.img_topic = rospy.get_param('~img_topic','/camera/rgb/image_raw')
        if self.img_transport == 'compressed':
            self.img_topic += '/compressed'

        # ROS
        self.img_subscriber = self.init_subscriber()
        self.class_pub = rospy.Publisher('~classification', String, queue_size=10)
        self.trigger = rospy.Service('~trigger_classification',
            SetBool, self.trigger_classification)

        #Classifier
        self.io_class = in_out_classifier();

        #CV2 bridge
        self.bridge = CvBridge()

    ## Initializes the image subscriber
    #  @param self The object pointer.
    def init_subscriber(self):

        if self.img_transport == 'compressed':
            rospy.logout("Indoor-Outdoor classifier started, using compressed image")
            return rospy.Subscriber(self.img_topic,
                CompressedImage, self.image_callback,  queue_size = 1)
        elif not self.img_transport == 'raw':
            rospy.logwarn("Wrong img_transport param. Using \"raw\".")

        rospy.logout("Indoor-Outdoor classifier started, using raw image")
        return rospy.Subscriber(self.img_topic,
            Image, self.image_callback,  queue_size = 1)


    ## Service to trigger classification
    #  @param self The object pointer.
    #  @param req SetBoolRequest
    #  @return SetBoolResponse Message, string for activation
    def trigger_classification(self, req):
        if(req.data):
            self.img_subscriber = self.init_subscriber()
            return SetBoolResponse(True, "enabled")
        else:
            self.img_subscriber.unregister()
            return SetBoolResponse(True, "disabled")

    ## Callback for Image
    #  @param self The object pointer.
    #  @param ros_data sensor_msgs/CompressedImage or sensor_msgs/Image message
    def image_callback(self, ros_data):

	rospy.sleep(1) #sleep for saving computation power
        if isinstance(ros_data, CompressedImage):
            #### Convert compressed image to CV2 ####
            nparr = np.fromstring(ros_data.data, np.uint8)
            image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            image = cv2.cvtColor(image , cv2.COLOR_BGR2RGB)
        elif isinstance(ros_data, Image):
            #### Convert raw image to CV2 ####
            try:
              image = self.bridge.imgmsg_to_cv2(ros_data, "bgr8")
            except CvBridgeError as e:
              print(e)
              return
        else:
            rospy.logerr("Message on topic "+self.img_topic+" has wrong type")
            return

        # Classify image
        if self.io_class.classify(image) == 0:
            self.class_pub.publish("indoor")
        else:
            self.class_pub.publish("outdoor")


def main(args):

    rospy.init_node('in_out_classifier_node', anonymous=False)
    io_class_node = in_out_classifier_node()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down Indoor Outdoor classifier"

if __name__ == '__main__':
    main(sys.argv)
