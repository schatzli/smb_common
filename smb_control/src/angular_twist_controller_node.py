#!/usr/bin/env python3  
import roslib
import rospy
import numpy
import math
import tf
import rospkg
import rosbag
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from copy import deepcopy



twistTopicIn = "/twist_in"
twistTopicOut = "/twist_out"
scalingFactorLin = 1.0
scalingFactorAng = 1.0
angularVelocityOffset = 0.7
angularVelocityDeadbandWidth = 0.2

imuTopic = "versavis/imu"
kp = 0.5
ki = 0.0
i_max = 3.0
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
        
    if integratedError > i_max:
        integratedError = i_max
    elif integratedError < -i_max:
        integratedError = -i_max
      
    proportionalAction = kp * (desired_twist.angular.z - yawRateMeas)
    integralAction = ki * integratedError
       
    correctedAngularRateCmd = proportionalAction + integralAction

    msgOut = deepcopy(desired_twist)
    msgOut.angular.z = correctedAngularRateCmd 
    
    rospy.loginfo_throttle(2, f"yaw_des: {desired_twist.angular.z} yawRateMeas: {yawRateMeas} int_error: {integratedError} dt: {dt}")
    
    correctedTwistPublisher.publish(msgOut)
    
    
if __name__ == '__main__':
    rospy.init_node('angular_twist_controller')

    kp = rospy.get_param("~kp")
    ki = rospy.get_param("~ki")
    i_max = rospy.get_param("~i_max")
    #angularVelocityOffset = rospy.get_param("~angular_z_offset")
    #angularVelocityDeadbandWidth = rospy.get_param("~angular_z_deadband_width")

    print("Launching the angular_twist_controller node.. ")
    #print("Linear scaling factor is %f " % scalingFactorLin)
    #print("Angular scaling factor is %f " % scalingFactorAng)
    #print("Angular velocity offset is %f " % angularVelocityOffset)
    #print("Angular velocity deadband width is %f " % angularVelocityDeadbandWidth)

    rospy.Subscriber(twistTopicIn, Twist, twistCallback)
    rospy.Subscriber(imuTopic, Imu, imuCallback)
    correctedTwistPublisher = rospy.Publisher(twistTopicOut, Twist, queue_size=10)
    
    rospy.spin()
