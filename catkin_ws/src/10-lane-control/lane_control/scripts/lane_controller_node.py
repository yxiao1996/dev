#!/usr/bin/env python
import rospy
import math
from duckietown_msgs.msg import Twist2DStamped, LanePose, Pose2DStamped
from geometry_msgs.msg import Pose2D
import skfuzzy as fuzzy
import skfuzzy.control as ctrl
import numpy as np
from fuzzy import FuzzyControl

class lane_controller(object):
    """fuzzy control version of lane controller"""
    def __init__(self):
        self.node_name = rospy.get_name()
        self.lane_reading = None

        self.pub_counter = 0

        # Initialize buffers
        self.last_timestamp = 0
        self.speed_v = 1.0
        self.speed_omega = 1.0

        self.omega_bar = 0.2
        # Initialize sparse universe
        universe = np.linspace(-2, 2, 40)

        # Create fuzzy variables
        lane_error = ctrl.Antecedent(universe, 'lane_error')
        head_error = ctrl.Antecedent(universe, 'head_error')
        out_v = ctrl.Consequent(np.linspace(0, 1, 40), 'out_v')
        out_omega = ctrl.Consequent(universe, 'out_omega')

        # Generate membership function automatically
        names = ['nb', 'ns', 'ze', 'ps', 'pb']
        #lane_error.automf(names=names)
        #head_error.automf(names=names)
        #out_v.automf(names=names)
        #out_omega.automf(names=names)

        # Input membership
        lane_error['nb'] = fuzzy.zmf(lane_error.universe, -0.3, -0.1)
        lane_error['ns'] = fuzzy.trimf(lane_error.universe, [-0.2, -0.1, 0.0])
        lane_error['ze'] = fuzzy.trimf(lane_error.universe, [-0.1, 0.0, 0.1])
        lane_error['ps'] = fuzzy.trimf(lane_error.universe, [0.0, 0.1, 0.2])
        lane_error['pb'] = 1.0 - fuzzy.zmf(lane_error.universe, 0.1, 0.3)
        head_error['nb'] = fuzzy.zmf(head_error.universe, -0.7, -0.3)
        head_error['ns'] = fuzzy.trimf(head_error.universe, [-0.4, -0.25, -0.1])
        head_error['ze'] = fuzzy.trimf(head_error.universe, [-0.2, 0.0, 0.2])
        head_error['ps'] = fuzzy.trimf(head_error.universe, [0.1, 0.25, 0.4])
        head_error['pb'] = 1.0 - fuzzy.zmf(head_error.universe, 0.3, 0.7)

        # Output membership
        out_omega['nb'] = fuzzy.zmf(out_omega.universe, -0.7, -0.3)
        out_omega['ns'] = fuzzy.trimf(out_omega.universe, [-0.4, -0.25, -0.1])
        out_omega['ze'] = fuzzy.trimf(out_omega.universe, [-0.2, 0.0, 0.2])
        out_omega['ps'] = fuzzy.trimf(out_omega.universe, [0.1, 0.25, 0.4])
        out_omega['pb'] = 1.0 - fuzzy.zmf(out_omega.universe, 0.3, 0.7)
        
        out_v['ps'] = fuzzy.zmf(out_omega.universe, 0.1, 0.4)
        out_v['pb'] = 1.0 - fuzzy.zmf(out_omega.universe, 0.3, 0.5)

        # Define rules
        # Rule 0: when perfectly in lane, run straight as high speed
        rule0 = ctrl.Rule(antecedent=((lane_error['ze'] & head_error['ze'])),
                          consequent=(out_v['pb']),
                          label='rule_0')
        rule1 = ctrl.Rule(antecedent=((lane_error['ze'] & head_error['ze'])),
                          consequent=(out_omega['ze']),
                          label='rule_1')
        
        # Rule 1: when slight heading error, adjust head slightly
        rule2 = ctrl.Rule(antecedent=((lane_error['ns'] & head_error['ns']) |
                                      (lane_error['ps'] & head_error['ns']) |
                                      (lane_error['nb'] & head_error['ns']) |
                                      (lane_error['pb'] & head_error['ns']) |
                                      (lane_error['ze'] & head_error['ns']) |
                                      (lane_error['ns'] & head_error['nb']) |
                                      (lane_error['ps'] & head_error['nb']) |
                                      (lane_error['nb'] & head_error['nb']) |
                                      (lane_error['pb'] & head_error['nb']) |
                                      (lane_error['ze'] & head_error['nb']) |
                                      (lane_error['ns'] & head_error['ps']) |
                                      (lane_error['ps'] & head_error['ps']) |
                                      (lane_error['nb'] & head_error['ps']) |
                                      (lane_error['pb'] & head_error['ps']) |
                                      (lane_error['ze'] & head_error['ps']) |
                                      (lane_error['ns'] & head_error['pb']) |
                                      (lane_error['ps'] & head_error['pb']) |
                                      (lane_error['nb'] & head_error['pb']) |
                                      (lane_error['pb'] & head_error['pb']) |
                                      (lane_error['ze'] & head_error['pb']) |
                                      (lane_error['ns'] & head_error['ze']) |
                                      (lane_error['ps'] & head_error['ze']) |
                                      (lane_error['nb'] & head_error['ze']) |
                                      (lane_error['pb'] & head_error['ze'])),
                          consequent=(out_v['ps']),
                          label='rule_2')
        #rule_v2 = ctrl.Rule(antecedent=((lane_error['ze'] & head_error['ze'])),
        #                  consequent=(out_v['ze']),
        #                  label='rule_v2')
        rule3 = ctrl.Rule(antecedent=((lane_error['ns'] & head_error['ns']) |
                                      (lane_error['ps'] & head_error['ns']) |
                                      (lane_error['nb'] & head_error['ns']) |
                                      (lane_error['pb'] & head_error['ns']) |
                                      (lane_error['ze'] & head_error['ns']) |
                                      (lane_error['ns'] & head_error['ze']) |
                                      (lane_error['nb'] & head_error['ze'])),
                          consequent=(out_omega['ps']),
                          label='rule_3')
        rule4 = ctrl.Rule(antecedent=((lane_error['ns'] & head_error['ps']) |
                                      (lane_error['ps'] & head_error['ps']) |
                                      (lane_error['nb'] & head_error['ps']) |
                                      (lane_error['pb'] & head_error['ps']) |
                                      (lane_error['ze'] & head_error['ps']) |
                                      (lane_error['ps'] & head_error['ze']) |
                                      (lane_error['pb'] & head_error['ze'])),
                          consequent=(out_omega['ns']),
                          label='rule_4')
        
        # Rule 2: when heading error is big, adjust heading seriously
        rule5 = ctrl.Rule(antecedent=((lane_error['ns'] & head_error['nb']) |
                                      (lane_error['ps'] & head_error['nb']) |
                                      (lane_error['nb'] & head_error['nb']) |
                                      (lane_error['pb'] & head_error['nb']) |
                                      (lane_error['ze'] & head_error['nb'])),
                          consequent=(out_omega['pb']),
                          label='rule_5')
        rule6 = ctrl.Rule(antecedent=((lane_error['ns'] & head_error['pb']) |
                                      (lane_error['ps'] & head_error['pb']) |
                                      (lane_error['nb'] & head_error['pb']) |
                                      (lane_error['pb'] & head_error['pb']) |
                                      (lane_error['ze'] & head_error['pb'])),
                          consequent=(out_omega['nb']),
                          label='rule_6')

        self.fuzzy_system = ctrl.ControlSystem(rules=[rule0, rule1, rule2, rule3, rule4, rule5, rule6])
        #self.sim = ctrl.ControlSystemSimulation(self.fuzzy_system, flush_after_run=1)
        ctl_sys = FuzzyControl()
        self.sim = ctl_sys.get_ctl()

        # Setup parameters
        self.setGains()

        # Publicaiton
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        self.pub_debug = rospy.Publisher("~debug", Pose2D, queue_size=1)

        # Subscriptions
        self.sub_lane_reading = rospy.Subscriber("~lane_pose", LanePose, self.cbPose, queue_size=1)
        self.sub_speed_factor = rospy.Subscriber("~speed", Pose2DStamped, self.cbSpeed, queue_size=1)

        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)

        # timer
        self.gains_timer = rospy.Timer(rospy.Duration.from_sec(1.0), self.getGains_event)
        rospy.loginfo("[%s] Initialized " %(rospy.get_name()))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbSpeed(self, msg):
        self.speed_v = msg.x
        self.speed_omega = msg.y
        print msg

    def setGains(self):
        v_bar = 0.5 # nominal speed, 0.5m/s
        k_theta = -2.0
        k_d = - (k_theta ** 2) / ( 4.0 * v_bar)
        theta_thres = math.pi / 6
        d_thres = math.fabs(k_theta / k_d) * theta_thres
        d_offset = 0.0

        self.v_bar = self.setupParameter("~v_bar",v_bar) # Linear velocity
        # FIXME: AC aug'17: are these inverted?
        self.k_d = self.setupParameter("~k_d",k_theta) # P gain for theta
        self.k_theta = self.setupParameter("~k_theta",k_d) # P gain for d
        self.d_thres = self.setupParameter("~d_thres",theta_thres) # Cap for error in d
        self.theta_thres = self.setupParameter("~theta_thres",d_thres) # Maximum desire theta
        self.d_offset = self.setupParameter("~d_offset",d_offset) # a configurable offset from the lane position

    def getGains_event(self, event):
        v_bar = rospy.get_param("~v_bar")
        k_d = rospy.get_param("~k_d")
        k_theta = rospy.get_param("~k_theta")
        d_thres = rospy.get_param("~d_thres")
        theta_thres = rospy.get_param("~theta_thres")
        d_offset = rospy.get_param("~d_offset")

        params_old = (self.v_bar,self.k_d,self.k_theta,self.d_thres,self.theta_thres, self.d_offset)
        params_new = (v_bar,k_d,k_theta,d_thres,theta_thres, d_offset)

        if params_old != params_new:
            rospy.loginfo("[%s] Gains changed." %(self.node_name))
            rospy.loginfo("old gains, v_var %f, k_d %f, k_theta %f, theta_thres %f, d_thres %f, d_offset %f" %(params_old))
            rospy.loginfo("new gains, v_var %f, k_d %f, k_theta %f, theta_thres %f, d_thres %f, d_offset %f" %(params_new))
            self.v_bar = v_bar
            self.k_d = k_d
            self.k_theta = k_theta
            self.d_thres = d_thres
            self.theta_thres = theta_thres
            self.d_offset = d_offset


    def custom_shutdown(self):
        rospy.loginfo("[%s] Shutting down..." %self.node_name)

        # Stop listening
        self.sub_lane_reading.unregister()

        # Send stop command
        car_control_msg = Twist2DStamped()
        car_control_msg.v = 0.0
        car_control_msg.omega = 0.0
        self.publishCmd(car_control_msg)
        rospy.sleep(0.5) #To make sure that it gets published.
        rospy.loginfo("[%s] Shutdown" %self.node_name)


    def publishCmd(self, car_cmd_msg):

        #wheels_cmd_msg = WheelsCmdStamped()
        #wheels_cmd_msg.header.stamp = stamp
        #speed_gain = 1.0
        #steer_gain = 0.5
        #vel_left = (speed_gain*speed - steer_gain*steering)
        #vel_right = (speed_gain*speed + steer_gain*steering)
        #wheels_cmd_msg.vel_left = np.clip(vel_left,-1.0,1.0)
        #wheels_cmd_msg.vel_right = np.clip(vel_right,-1.0,1.0)

        self.pub_car_cmd.publish(car_cmd_msg)
        #self.pub_wheels_cmd.publish(wheels_cmd_msg)

    def cbPose(self, lane_pose_msg):

        self.lane_reading = lane_pose_msg

        # Calculating the delay image processing took
        timestamp_now = rospy.Time.now()
        image_delay_stamp = timestamp_now - self.lane_reading.header.stamp

        # delay from taking the image until now in seconds
        image_delay = image_delay_stamp.secs + image_delay_stamp.nsecs/1e9

        # Calculating time delay since last frame of image
        if self.last_timestamp == 0:
            delay_time = 100
        else:
            delay_time = (timestamp_now - self.last_timestamp)
            delay_time = delay_time.secs

        cross_track_err = lane_pose_msg.d - self.d_offset
        heading_err = lane_pose_msg.phi

        if math.fabs(cross_track_err) > self.d_thres:
            cross_track_err = cross_track_err / math.fabs(cross_track_err) * self.d_thres
        #car_control_msg.omega =  self.k_d * cross_track_err + self.k_theta * heading_err #*self.steer_gain #Right stick H-axis. Right is negative

        self.sim.input['lane_error'] = cross_track_err
        self.sim.input['head_error'] = heading_err
        self.sim.compute()
        ctl_omega = self.sim.output['out_omega']
        ctl_v = self.sim.output['out_v']
        #print ctl_v, ctl_omega

        # publish car command
        car_control_msg = Twist2DStamped()
        car_control_msg.header = lane_pose_msg.header
        car_control_msg.v = ctl_v*0.7*self.speed_v
        car_control_msg.omega = ctl_omega*2.0*self.speed_omega
        self.publishCmd(car_control_msg)

        # Update timestamp buffer
        self.last_timestamp = rospy.Time.now()
        
        # debuging
        # self.pub_counter += 1
        # if self.pub_counter % 50 == 0:
        #     self.pub_counter = 1
        #     print "lane_controller publish"
        #     print car_control_msg

if __name__ == "__main__":
    rospy.init_node("lane_controller",anonymous=False)
    lane_control_node = lane_controller()
    rospy.spin()
