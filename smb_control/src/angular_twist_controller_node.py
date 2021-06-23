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



twistTopicIn = "/twist_in"
twistTopicOut = "/twist_out"
scalingFactorLin = 1.0
scalingFactorAng = 1.0
angularVelocityOffset = 0.7
angularVelocityDeadbandWidth = 0.2

imuTopic = "versavis/imu"
yawRateMeas = 0.0
#yawRateDes = 0.0
kp = 0.5
ki = 0.0
i_max = 3.0
timestampPrevTwistMsg = 0.0
twistMsgTimeout = 0.5
integratedError = 0.0
printCounter = 0
    

def twistCallback(msg):
    global timestampPrevTwistMsg
    global integratedError
    global printCounter
    twistMsgTimeout
    yawRateMeas

    dt = rospy.get_time() - timestampPrevTwistMsg
    yawRateDes = msg.angular.z
    
    if dt > twistMsgTimeout:
        integratedError = 0.0
        print("Time since last twist msg was too long. Resetting integrated twist to 0.0..")
    else:
        integratedError += dt * (yawRateDes - yawRateMeas)
        
    if integratedError > i_max:
        integratedError = i_max
    elif integratedError < -i_max:
        integratedError = -i_max
    
    timestampPrevTwistMsg = rospy.get_time()
    
    feedForwardAction = msg.angular.z
    proportionalAction = kp * (yawRateDes - yawRateMeas)
    integralAction = ki * integratedError
    
    if integralAction > 1.0:
        integralAction = 1.0
        print("integral action clipped to 1.0")
    elif integralAction < -1.0:
        integralAction = -1.0
        print("integral action clipped to -1.0")
    
   
    if yawRateMeas == 0.0:
        print("Seems like imu msgs are not being received. Forwarding original twist commands..")
        correctedAngularRateCmd = feedForwardAction
    else:
        correctedAngularRateCmd = feedForwardAction + proportionalAction + integralAction

    msgOut = msg
    msgOut.angular.z = correctedAngularRateCmd 
    
    if printCounter == 10:
        print("yaw_des: ", yawRateDes, "yawRateMeas: ", yawRateMeas, "int_error: ", integratedError, "dt: ", dt)
        printCounter = 0
        
    printCounter += 1
    
    correctedTwistPublisher.publish(msgOut)
    
    
def imuCallback(msg):
    global yawRateMeas
    yawRateMeas = msg.angular_velocity.z
    # apply filter?
    
    
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
