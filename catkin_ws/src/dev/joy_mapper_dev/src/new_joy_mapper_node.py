#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Int8, String
import time
from std_srvs.srv import EmptyRequest, EmptyResponse, Empty
from duckietown_msgs.msg import BoolStamped, Twist2DStamped, FSMState

# Button List index of joy.buttons array:
# a = 0, b=1, x=2. y=3, lb=4, rb=5, back = 6, start =7, logitek = 8, left joy = 9, right joy = 10
class NewJoyMapper(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))
        self.state = 'init'
        self.joy = None
        self.last_pub_msg = None
        self.last_pub_time = rospy.Time.now()

        self.pub_go= rospy.Publisher("~letsgo", BoolStamped, queue_size=1)
        
        self.sub_joy_ = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size=1)
        self.sub_mode = rospy.Subscriber('/duckiebot0/fsm_node/mode', FSMState, self.cbMode)
        self.forward = rospy.ServiceProxy('/duckiebot0/open_loop_intersection_control_node/turn_forward',Empty)
        self.button2command = {
             # 'a' is pressed
            # 0: 'CAR_SIGNAL_A',
            # 'b' is pressed
            1: "COORDINATION",
            # 'Y' is pressed
            # 3: 'CAR_SIGNAL_C',
            # 'X' is pressed
            # 2: 'light_off',
            # lb is pressed
            # 4: 'traffic_light_go',
            # rb is pressed
            # 5: 'traffic_light_stop',
            # logitek button is pressed
            # 8: 'test_all_1',
        }
    def cbMode(self, msg):
        self.state = msg.state

    def cbJoy(self, joy_msg):
        self.joy = joy_msg
        self.publishControl()

    def publishControl(self):

        for b, state in self.button2command.items():
            if self.joy.buttons[b] == 1:
                if self.state == state:
                    #self.forward()
                    go_msg = BoolStamped()
                    go_msg.header.stamp = self.joy.header.stamp
                    go_msg.data = True
                    self.pub_go.publish(go_msg)
                    rospy.loginfo("Publishing forward command")

if __name__ == "__main__":
    rospy.init_node("new_joy_mapper",anonymous=False)
    led_joy_mapper = NewJoyMapper()
    rospy.spin()##  
# 