#!/usr/bin/env python
import rospy
from std_msgs.msg import String #Imports msg
from sensor_msgs.msg import CompressedImage
from duckietown_utils.bag_logs import numpy_from_ros_compressed
import numpy as np
from SSD import ssdDetector
import cv2
import time

class SsdDetector_node(object):
    def __init__(self):
        self.active = True
        # Save the name of the node
        self.node_name = rospy.get_name()
        
        rospy.loginfo("[%s] Initialzing." %(self.node_name))
        
        self.detector = self.detector = ssdDetector()
        self.status = "init"
        # Setup publishers
        #self.pub_topic_a = rospy.Publisher("~topic_a",String, queue_size=1)
        # Setup subscriber
        self.sub_cam = rospy.Subscriber("/duckiebot0/camera_node/image/compressed", CompressedImage, self.camera_callback)
        # Read parameters
        self.pub_timestep = self.setupParameter("~pub_timestep",0.2)
        # Create a timer that calls the infer_callback function every 1.0 second
        self.timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.infer_callback)

        rospy.loginfo("[%s] Initialzed." %(self.node_name))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def camera_callback(self,msg):
        if not self.active:
            return

        #float_time = msg.header.stamp.to_sec()
        
        rgb = numpy_from_ros_compressed(msg)
        self.status = "default"
        self.rgb = rgb

        rospy.loginfo("[%s] get image data" %(self.node_name))

    def infer_callback(self,event):
        # infering
        if self.status == "default":
            self.detector.detect(self.rgb)

    def on_shutdown(self):
        self.detector.onShutdown()
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('ssd_node', anonymous=False)

    # Create the NodeName object
    node = SsdDetector_node()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    
    # Keep it spinning to keep the node alive
    rospy.spin()