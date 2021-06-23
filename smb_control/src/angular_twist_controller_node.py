#!/usr/bin/env python3  
import roslib
import rospy
import numpy
import math
import tf
import rospkg
import rosbag
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from copy import deepcopy


ddynrec = DDynamicReconfigure("")

twistTopicIn = "/twist_in"
twistTopicOut = "/twist_out"
scalingFactorLin = 1.0
scalingFactorAng = 1.0
angularVelocityOffset = 0.7
angularVelocityDeadbandWidth = 0.2

imuTopic = "versavis/imu"
kp = 5.0
ki = 5.0
i_max = 5.0
max_yaw_output = 2.0
timestampPrevTwistMsg = 0.0
twistMsgTimeout = 0.5
imuMsgTimeout = 0.05
integratedError = 0.0

desired_twist = Twist()
last_yawRateDes_received = 0
last_imu_received = 0
integratedError = 0
imu_received = False

def twistCallback(msg):
    global desired_twist
    global imu_received
    global last_yawRateDes_received

    desired_twist = msg
    last_yawRateDes_received = rospy.get_time()
    if not imu_received:
        rospy.logwarn_throttle(5.0, "No imu data received yet, forward incoming twist.")
        correctedTwistPublisher.publish(msg)
    
def imuCallback(msg):
    global imu_received

    imu_received = True

    yawRateMeas = msg.angular_velocity.z
    # apply filter?

    global timestampPrevTwistMsg
    global integratedError
    global desired_twist

    imu_received = rospy.get_time()
    dt = imu_received - timestampPrevTwistMsg
    timestampPrevTwistMsg = imu_received

    if imu_received - last_yawRateDes_received > twistMsgTimeout:
        integratedError = 0.0
        yawRateMeas = 0.0
        desired_twist = Twist()
        rospy.logwarn_throttle(2.0, f"No desired twist received for {imu_received - last_yawRateDes_received} seconds. Command 0.0 velocity.")


    if dt > imuMsgTimeout:
        integratedError = 0.0
        rospy.logwarn_throttle(2.0, "Time since last imu msg was too long. Resetting integrated twist to 0.0.")
        return
    else:
        integratedError += dt * (desired_twist.angular.z - yawRateMeas)
        
    integratedError = numpy.clip(integratedError, -i_max, i_max)
      
    proportionalAction = kp * (desired_twist.angular.z - yawRateMeas)
    integralAction = ki * integratedError
       
    correctedAngularRateCmd = proportionalAction + integralAction

    correctedAngularRateCmd = numpy.clip(correctedAngularRateCmd, -max_yaw_output, max_yaw_output)

    msgOut = deepcopy(desired_twist)
    msgOut.angular.z = correctedAngularRateCmd 
    
    rospy.loginfo_throttle(2, f"yaw_des: {desired_twist.angular.z} yawRateMeas: {yawRateMeas} int_error: {integratedError} dt: {dt}")
    
    correctedTwistPublisher.publish(msgOut)

def dyn_rec_callback(config, level):
    rospy.loginfo("Received reconf call: " + str(config))
    global kp, ki, i_max, max_yaw_output
    kp = config["kp"]
    ki = config["ki"]
    i_max = config["i_max"]
    max_yaw_output = config["max_yaw_output"]
    return config    
    
if __name__ == '__main__':
    rospy.init_node('angular_twist_controller')

    kp = rospy.get_param("~kp", kp)
    ki = rospy.get_param("~ki", ki)
    i_max = rospy.get_param("~i_max", i_max)
    max_yaw_output = rospy.get_param("~max_yaw_output", max_yaw_output)

    ddynrec.add_variable("kp", "float/double variable", kp, 0.0, 10.0)
    ddynrec.add_variable("ki", "float/double variable", ki, 0.0, 10.0)
    ddynrec.add_variable("i_max", "float/double variable", i_max, 0.0, 5.0)
    ddynrec.add_variable("max_yaw_output", "float/double variable", max_yaw_output, 0.0, 5.0)
    ddynrec.start(dyn_rec_callback)

    print("Launching the angular_twist_controller node.. ")

    rospy.Subscriber(twistTopicIn, Twist, twistCallback, queue_size=1)
    rospy.Subscriber(imuTopic, Imu, imuCallback, queue_size=1)
    correctedTwistPublisher = rospy.Publisher(twistTopicOut, Twist, queue_size=1)
    
    rospy.spin()
