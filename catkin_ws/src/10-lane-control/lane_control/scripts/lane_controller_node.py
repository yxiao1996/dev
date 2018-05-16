#!/usr/bin/env python
import rospy
import math
from duckietown_msgs.msg import Twist2DStamped, LanePose, Pose2DStamped,  BoolStamped, AprilTagDetectionArray
from geometry_msgs.msg import Pose2D
import skfuzzy as fuzzy
import skfuzzy.control as ctrl
import numpy as np
from fuzzy import FuzzyControl
import copy
from std_srvs.srv import EmptyRequest, EmptyResponse, Empty

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
        self.april_factor = 1.0

        self.param_list = {
                       "turn_left":[ #time, velocity, angular vel
                       [0.8, 0.0, 0],
                       [1.8, 0.0, 2.896],
                       [0.8, 0.0, 0.0]
                       ],
                       "turn_right":[
                        [0.8, 0.0, 0],
                        [1.8, 0.0, -2.896],
                        [0.8, 0.0, 0.0]
                        ],
                        "turn_forward":[
                        [0.8, 0.43, 0.4],
                        [1.0, 0.43, 0.0],
                        [1.0, 0.0, 0.0],
                        [0.5, 0.0, 0.0]
                        ],
                        "turn_backward":[
                        [0.8, -0.43, 0.4],
                        [1.0, -0.43, 0.0],
                        [1.0, 0.0, 0.0],
                        [0.5, 0.0, 0.0]
                        ]
                }
        self.maneuvers = dict()

        self.maneuvers[0] = self.getManeuver("turn_left")
        self.maneuvers[1] = self.getManeuver("turn_forward")
        self.maneuvers[2] = self.getManeuver("turn_right")
        self.maneuvers[3] = self.getManeuver("turn_backward")

        self.srv_stop = rospy.Service("~stop", Empty, self.cbSrvStop)
        self.srv_turn_left = rospy.Service("~turn_left", Empty, self.cbSrvLeft)
        self.srv_turn_right = rospy.Service("~turn_right", Empty, self.cbSrvRight)
        self.srv_turn_forward = rospy.Service("~turn_forward", Empty, self.cbSrvForward)
        self.srv_turn_backward = rospy.Service("~turn_backward", Empty, self.cbSrvBackward)
        
        ctl_sys = FuzzyControl()
        self.sim = ctl_sys.get_ctl()

        # Setup parameters
        self.setGains()

        # Publicaiton
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        self.pub_move_cmd = rospy.Publisher("~move_cmd", Twist2DStamped, queue_size=1)
        self.pub_done = rospy.Publisher("~move_done",BoolStamped,queue_size=1)
        self.pub_debug = rospy.Publisher("~debug", Pose2D, queue_size=1)

        # Subscriptions
        self.sub_lane_reading = rospy.Subscriber("~lane_pose", LanePose, self.cbPose, queue_size=1)
        self.sub_speed_factor = rospy.Subscriber("~speed", Pose2DStamped, self.cbSpeed, queue_size=2)
        #self.sub_mode = rospy.Subscriber("~mode", FSMState, self.cbFSMState, queue_size=1)
        self.sub_prePros = rospy.Subscriber("~apriltags_in", AprilTagDetectionArray, self.cbApril, queue_size=1)
        self.rate = rospy.Rate(30)
        
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

    #def cbFSMState(self,msg):
        # self.mode = msg.state

    def getManeuver(self,param_name):
        param_list = self.param_list[param_name]
        # rospy.loginfo("PARAM_LIST:%s" %param_list)        
        maneuver = list()
        for param in param_list:
            maneuver.append((param[0],Twist2DStamped(v=param[1],omega=param[2])))
        # rospy.loginfo("MANEUVER:%s" %maneuver)
        return maneuver

    def cbApril(self, msg):
        d = msg.detections
        if len(d) > 0:
            print "detect vehicle"
            self.april_factor = 0.0
        else:
            self.april_factor = 1.0
            
    def publishDoneMsg(self):
        msg = BoolStamped()
        msg.header.stamp = rospy.Time.now()
        msg.data = True
        self.pub_done.publish(msg)
        rospy.loginfo("[%s] move_done!" %(self.node_name))

    def cbSrvStop(self,req):
        self.trigger(-1)
        return EmptyResponse()

    def cbSrvLeft(self,req):
        self.trigger(0)
        return EmptyResponse()
    
    def cbSrvForward(self,req):
        self.trigger(1)
        return EmptyResponse()

    def cbSrvBackward(self,req):
        self.trigger(3)
        return EmptyResponse()    
    
    def cbSrvRight(self,req):
        self.trigger(2)
        return EmptyResponse()
    
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

        #self.sim.input['lane_error'] = cross_track_err
        #self.sim.input['head_error'] = heading_err
        #self.sim.compute()
        #ctl_omega = self.sim.output['out_omega']
        #ctl_v = self.sim.output['out_v']
        #print ctl_v, ctl_omega

        # publish car command
        car_control_msg = Twist2DStamped()
        car_control_msg.header = lane_pose_msg.header
        car_control_msg.v = 0.35 * self.april_factor
        car_control_msg.omega = 1.0*(self.k_d * cross_track_err + self.k_theta * heading_err) * self.april_factor
        #car_control_msg.v = ctl_v*0.7*self.speed_v
        #car_control_msg.omega = ctl_omega*2.0*self.speed_omega
        
        self.publishCmd(car_control_msg)

        # Update timestamp buffer
        self.last_timestamp = rospy.Time.now()
        
        # debuging
        # self.pub_counter += 1
        # if self.pub_counter % 50 == 0:
        #     self.pub_counter = 1
        #     print "lane_controller publish"
        #     print car_control_msg
        
    def trigger(self,turn_type):
        if turn_type == -1: #Wait. Publish stop command. Does not publish done.
            cmd = Twist2DStamped(v=0.0,omega=0.0)
            cmd.header.stamp = rospy.Time.now()
            self.pub_cmd.publish(cmd)
            return

        published_already = False
        for index, pair in enumerate(self.maneuvers[turn_type]):
            cmd = copy.deepcopy(pair[1])
            start_time = rospy.Time.now()
            end_time = start_time + rospy.Duration.from_sec(pair[0])
            while rospy.Time.now() < end_time:
                #if not self.mode == "AT_GOAL": # If not in the mode anymore, return
                    #return
                cmd.header.stamp = rospy.Time.now()
                self.pub_move_cmd.publish(cmd)
                if index > 1:
                    # See if need to publish interesction_done
                    if not (published_already):
                        published_already = True
                        self.publishDoneMsg()
                        return
                self.rate.sleep()
        # Done with the sequence
        if not published_already:
            self.publishDoneMsg()

if __name__ == "__main__":
    rospy.init_node("lane_controller",anonymous=False)
    lane_control_node = lane_controller()
    rospy.spin()
