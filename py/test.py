#!/usr/bin/python

import rospy
import std_msgs.msg
import geometry_msgs.msg
 
class RosConnector:
    def __init__(self, prefix):
        # Get the ~private namespace parameters from command line or launch file.
        
        self.pubLand = rospy.Publisher("/"+prefix+"/ardrone/land", std_msgs.msg.Empty)
        self.pubReset = rospy.Publisher("/"+prefix+"/ardrone/reset", std_msgs.msg.Empty)
        self.pubSetpoint = rospy.Publisher("/"+prefix+"/setpoint", geometry_msgs.msg.Pose)
        
        # Set the message to publish as our custom message.
        # Initialize message variables.
        print "Init done"
 
    def takeOff(self):
        pass
        
        

def takeOff():
        ros.takeOff()

if __name__ == "__main__":
    
    ros = RosConnector("ardrone")
    pubTakeoff = rospy.Publisher("/ardrone"+"/ardrone/takeoff", std_msgs.msg.Empty, queue_size=1)
    rospy.init_node('controller_'+"ardrone")
    msg = std_msgs.msg.Empty()
    pubTakeoff.publish(msg)

