#!/usr/bin/env python

import sys
import time
from gi.repository import Gtk, cairo, Pango, PangoCairo, GObject, Gdk
import threading

import rospy
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import tf.transformations
import rospkg
import math
from ram.msg import nonlinearity
from ram.msg import gains

import os
import xmlrpclib
import socket

import subprocess

class RosConnector:
    def __init__(self, prefix, ip):
        self.pubTakeoff = rospy.Publisher("/"+prefix+"/ardrone/takeoff", std_msgs.msg.Empty, queue_size=1)
        self.pubLand = rospy.Publisher("/"+prefix+"/ardrone/land", std_msgs.msg.Empty, queue_size=1)
        self.pubReset = rospy.Publisher("/"+prefix+"/ardrone/reset", std_msgs.msg.Empty, queue_size=1)
        self.pubSetpoint = rospy.Publisher("/"+prefix+"/setpoint", geometry_msgs.msg.Pose)
        self.pubCmd = rospy.Publisher("/"+prefix+"/cmd_vel", geometry_msgs.msg.Twist)
        self.pubNonLinear = rospy.Publisher("/"+prefix+"/nonlinearity", nonlinearity)
        self.pubGains = rospy.Publisher("/"+prefix+"/gains", gains)

        self.subJoystick = rospy.Subscriber("/"+prefix+"/cmd_vel_joy", geometry_msgs.msg.Twist, self.joyCB)
        self.subController = rospy.Subscriber("/"+prefix+"/cmd_vel_controller", geometry_msgs.msg.Twist, self.controllerCB)
        self.subJoyRaw = rospy.Subscriber("/joy", sensor_msgs.msg.Joy, self.joyRawCB, queue_size=1) 

        self.subSetpoint = rospy.Subscriber("/"+prefix+"/setpoint", geometry_msgs.msg.Pose, self.setpointCB)
        self.setpoint = geometry_msgs.msg.Pose()

        self.pr = subprocess.Popen(["roslaunch","ram", "controller_module.launch", "prefix:="+prefix, "ip:="+ip])
        self.massConnectorSubscriberDict = {}
        self.massConnectorPositionDict = {}


    def joyCB(self, msg):
        global publishJoystick
        if publishJoystick:
            self.pubCmd.publish(msg)

    def setpointCB(self, msg):
        Gdk.threads_enter()
        self.setpoint = msg
        Gdk.threads_leave()


    def joyRawCB(self, msg):
        Gdk.threads_enter()
        if window.is_active():
            if msg.buttons[3] == 1: # Change button to 3 for PS3, 8 for Joystick btn 9
                toggleJoystickCheckbox()
                time.sleep(0.3)
        Gdk.threads_leave()


    def controllerCB(self, msg):
        global publishJoystick
        global publishZero
        global publishNothing

        if not publishJoystick and not publishZero and not publishNothing:
            self.pubCmd.publish(msg)
        if publishNothing and not publishJoystick:
            # Publish a non hover
            msg = geometry_msgs.msg.Twist()
            msg.angular.x = 1;
            msg.angular.y = 1;
            self.pubCmd.publish(msg)
        if publishZero and not publishNothing and not publishJoystick:
            self.pubCmd.publish(geometry_msgs.msg.Twist())
    
    def clean(self):
        print "Cleaning!"
        try:
            self.pr.terminate()
        except (OSError, AttributeError):
            # can't kill a dead/non existent proc
            pass

    def takeOff(self):
        msg = std_msgs.msg.Empty()
        self.pubTakeoff.publish(msg)

    def land(self):
        msg = std_msgs.msg.Empty()
        self.pubLand.publish(msg)

    def reset(self):
        msg = std_msgs.msg.Empty()
        self.pubReset.publish(msg)

    def setSetpoint(self, x, y, z, yaw):
        msg = geometry_msgs.msg.Pose()
        msg.position.x = x
        msg.position.y = y;
        msg.position.z = z;

        quaternion = tf.transformations.quaternion_from_euler(0, 0, (yaw*2*math.pi)/(360))
        msg.orientation.x = quaternion[0]
        msg.orientation.y = quaternion[1]
        msg.orientation.z = quaternion[2]
        msg.orientation.w = quaternion[3]
        self.pubSetpoint.publish(msg)

    def addMassListener(self, prefix):
        self.massConnectorSubscriberDict[prefix] = rospy.Subscriber("/"+prefix+"/unfiltered_pose", geometry_msgs.msg.Pose, self.massCB, callback_args=prefix)

    def massCB(self, pose, prefix):
        self.massConnectorPositionDict[prefix] = pose

    def getMassConnectorLocation(self, prefix):
        return self.massConnectorPositionDict[prefix]

    def setNonLinear(self, x, y, z, xoff, yoff):
        msg = nonlinearity()
        msg.x = x
        msg.y = y
        msg.z = z
        msg.xoff = xoff
        msg.yoff = yoff
        self.pubNonLinear.publish(msg)

    def setGains(self, p_z, d_z, p_rot, d_rot, p_trans, d_trans, i_action, v_damping, i_enabled, v_enabled):
        msg = gains()
        msg.p_z = p_z
        msg.d_z = d_z
        msg.p_rot = p_rot
        msg.d_rot = d_rot
        msg.p_trans = p_trans
        msg.d_trans = d_trans
        msg.i_action = i_action
        msg.v_damping = v_damping
        msg.i_enabled = i_enabled
        msg.v_enabled = v_enabled
        self.pubGains.publish(msg)

def btnClose(widget, event):
    ros.clean()

    try:
        joyThread.poll()
        if(joyThread.returncode == None):
            joyThread.terminate()
    except (NameError, AttributeError):
        pass

    Gtk.main_quit(widget, event)

def btnTakeOff(b):
    ros.takeOff()

def btnLand(b):
    ros.land()

def btnReset(b):
    ros.reset()

def boxJoystick(box):
    global publishJoystick
    publishJoystick = box.get_active()

def boxSetpoint(box):
    global publishSetpoint
    publishSetpoint = box.get_active()
    if publishSetpoint == True:
        setSetpoint()

def boxZero(box):
    global publishZero
    publishZero = box.get_active()

def boxNothing(box):
    global publishNothing
    publishNothing = box.get_active()

def scaleX(scale):
    setSetpoint()

def scaleY(scale):
    setSetpoint()

def scaleZ(scale):
    setSetpoint()

def scaleYaw(scale):
    setSetpoint()

def setSetpoint():
    global publishSetpoint
    setx = builder.get_object("scalestoreX").get_value()
    sety = builder.get_object("scalestoreY").get_value()
    setz = builder.get_object("scalestoreZ").get_value()
    setyaw = builder.get_object("scalestoreYaw").get_value()
    if publishSetpoint:
        ros.setSetpoint(setx, sety, setz, setyaw)

def cmbMassConnectors(cmb):
    pass

def btnSetpointFromMassConnector(btn):
    active = builder.get_object("cmbMassConnectors").get_active()
    text = builder.get_object("listStoreMassConnectors")[active][0]
    pose = ros.getMassConnectorLocation(text)
    
    builder.get_object("scalestoreX").set_value(pose.position.x+0.3)
    builder.get_object("scalestoreY").set_value(pose.position.y)
    builder.get_object("scalestoreZ").set_value(pose.position.z+0.27)
    builder.get_object("scalestoreYaw").set_value(0)

def btnHookForward(btn):
    builder.get_object("scalestoreX").set_value(builder.get_object("scalestoreX").get_value()-0.45)
    builder.get_object("scalestoreZ").set_value(builder.get_object("scalestoreZ").get_value()+0.05)

def btnSetNonLinear(btn):
    x = builder.get_object("scalestoreNonLinX").get_value()
    y = builder.get_object("scalestoreNonLinY").get_value()
    z = builder.get_object("scalestoreNonLinZ").get_value()
    offx = builder.get_object("scalestoreOffsetX").get_value()
    offy = builder.get_object("scalestoreOffsetY").get_value()
    ros.setNonLinear(x,y,z,offx,offy)

def btnGains(btn):
    p_z = builder.get_object("scalestoreGainPZ").get_value()
    d_z = builder.get_object("scalestoreGainDZ").get_value()
    p_rot = builder.get_object("scalestoreGainPRot").get_value()
    d_rot = builder.get_object("scalestoreGainDRot").get_value()
    p_trans = builder.get_object("scalestoreGainPTrans").get_value()
    d_trans = builder.get_object("scalestoreGainDTrans").get_value()
    i_action = builder.get_object("scalestoreGainI").get_value()
    v_damping = builder.get_object("scalestoreGainVel").get_value()
    i_enabled = builder.get_object("boxIAction").get_active()
    v_enabled = builder.get_object("boxVelDamping").get_active()
    ros.setGains(p_z, d_z, p_rot, d_rot, p_trans, d_trans, i_action, v_damping, i_enabled, v_enabled)

def toggleJoystickCheckbox():
    currentstate = builder.get_object("boxJoystick").get_active()
    if currentstate:
        builder.get_object("boxJoystick").set_active(False)
    else:
        builder.get_object("boxJoystick").set_active(True)

def draw(w, d):
    pass

def readSetpointFromConnector():
    if builder.get_object("boxLive").get_active():
        builder.get_object("scaleX").set_value(ros.setpoint.position.x)
        builder.get_object("scaleY").set_value(ros.setpoint.position.y)
        builder.get_object("scaleZ").set_value(ros.setpoint.position.z)
        euler = tf.transformations.euler_from_quaternion([ros.setpoint.orientation.x, ros.setpoint.orientation.y, ros.setpoint.orientation.z, ros.setpoint.orientation.w])
        builder.get_object("scaleYaw").set_value(euler[2])
    return True

if __name__ == "__main__":

    m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
    try:
        code, msg, val = m.getSystemState('controller_instance')
    except socket.error:
        print "Please start roscore first."
        sys.exit()

    # This thing should only be started with an IP address and prefix as arguments.
    if len(sys.argv) < 4:
        print "Please only initiate controllers from the interface."
        sys.exit()
    else:
        prefix = sys.argv[1]
        ip = sys.argv[2]
        massConnectors = int(sys.argv[3])


    publishSetpoint = False
    publishJoystick = False
    publishZero = False
    publishNothing = False
    previoustime = 0


    rospy.init_node('controller_'+prefix)

    GObject.timeout_add(250, readSetpointFromConnector)
    GObject.threads_init()
    Gdk.threads_init()

    rospack = rospkg.RosPack()
    package_location = rospack.get_path("ram")
    gladefile = package_location+"/py/controller.glade"
    builder = Gtk.Builder()
    builder.add_from_file(gladefile)

    # Init ros
    ros = RosConnector(prefix, ip)

    # Add mass connectors to list
    connector = 0
    store = builder.get_object("listStoreMassConnectors")
    
    while connector != massConnectors:
        connector = connector + 1
        
        ros.addMassListener("massConnector"+str(connector))
        store.append(["massConnector"+str(connector)])

    handlers = {
        "quit": btnClose,
        "btnTakeOff": btnTakeOff,
        "btnLand": btnLand,
        "btnReset": btnReset,
        "boxJoystick": boxJoystick,
        "boxSetpoint": boxSetpoint,
        "boxZero": boxZero,
        "boxNothing": boxNothing,
        "scaleX": scaleX,
        "scaleY": scaleY,
        "scaleZ": scaleZ,
        "scaleYaw": scaleYaw,
        "draw": draw,
        "cmbMassConnectors": cmbMassConnectors,
        "btnSetpointFromMassConnector": btnSetpointFromMassConnector,
        "btnHookForward": btnHookForward,
        "btnSetNonLinear": btnSetNonLinear,
        "btnGains": btnGains
    }
    builder.connect_signals(handlers)

    

    window = builder.get_object("window1")
    window.set_title(prefix + " ("+ip+")")
    window.show_all()

    Gdk.threads_enter()
    Gtk.main()
    Gdk.threads_leave()