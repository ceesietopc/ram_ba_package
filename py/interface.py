#!/usr/bin/python

import sys
import json
import yaml
import dbus
import time
import telnetlib
from gi.repository import Gtk, GObject

import rospy
import rospkg

import threading
import subprocess
import multiprocessing

import os
import xmlrpclib
import socket

SSID = "RAM_drones"
SSID_SLUG = "ardrone"
wlan_interface = "wlan0"
device_path = None
connection_path = None
settings_path = None
controllerProcesses = []
connectors = 0

class controllerThread(threading.Thread):
    def __init__(self, ssid, onExit, popenArgs):
        self.stdout = None
        self.stderr = None
        threading.Thread.__init__(self)
        self.daemon = True
        self.ssid = ssid
        self.onExit = onExit
        self.popenArgs = popenArgs

    def run(self):
        p = subprocess.Popen(self.popenArgs,
                             shell=False)
        p.wait()
        self.onExit(self.ssid)

def importDrones(buttonPressed):
    # Start of with getting a list of drone objects from a JSON file
    json_data = open(package_location+"/py/drones.json").read()
    data = json.loads(json_data)

    # IP Address should can change everytime. SSID, Prefix and trackable_id should remain the same
    store = builder.get_object("liststore1")
    store.clear()
    for drone in data:
        store.append([drone['ssid'], "",drone['prefix'], drone['trackable_id'], "", ""])
    optiTrackUpdate()

def exportDrones(buttonPressed):
    drones = [];
    store = builder.get_object("liststore1")
    if len(store) < 1:
        return False

    for drone in store:
        drones.append({'ssid': drone[0], 'prefix': drone[2], 'trackable_id': drone[3]})
    with open(package_location+'/py/drones.json','w') as outfile:
            json.dump(drones,outfile)

def getNextIP():
    ip = 9;
    inuse = True
    store = builder.get_object("liststore1")

    while inuse:
        inuse = False
        for drone in store:
            if drone[1] == "192.168.1."+str(ip):
                inuse = True
                ip += 1
                continue
    return ip

def assignIP(buttonPressed):
    # Loop through store
    store = builder.get_object("liststore1")
    # TODO: Replace this value by the highest IP in the list + 1. Or even better: smallest available one.
    for drone in store:
        if (drone[1] == "") and (drone[4] == "Yes"):
            ip = str(getNextIP())
            print "Connecting!"
            droneConnect(drone[0])
            tn = telnetlib.Telnet("192.168.1.1")
            tn.read_some()

            tn.write("rm /data/wifi.sh\n")
            tn.read_some()
            tn.write("echo \"killall udhcpd\" > /data/wifi.sh\n")
            tn.read_some()
            tn.write("echo \"ifconfig ath0 down\" >> /data/wifi.sh\n")
            tn.read_some()
            tn.write("echo \"iwconfig ath0 mode managed essid "+SSID+"\" >> /data/wifi.sh\n")
            tn.read_some()
            tn.write("echo \"ifconfig ath0 192.168.1."+ip+" netmask 255.255.255.0 up\" >> /data/wifi.sh\n")
            tn.read_some()
            tn.write("chmod +x /data/wifi.sh\n");
            tn.read_some()
            tn.write("sh /data/wifi.sh\n")
            # IMPORTANT At this point, we lose connectivity
            #tn.write("exit\n")

            disconnect()
            drone[1] = "192.168.1."+ip
            drone[4] = "No"

def findNetworks():
    drones = []
    global device_path

    # Enable Wireless. If Wireless is already enabled, this does nothing.
    was_wifi_enabled = manager_props.Get("org.freedesktop.NetworkManager",
                                         "WirelessEnabled")
    if not was_wifi_enabled:
        print "Enabling WiFi and sleeping for 10 seconds ..."
        manager_props.Set("org.freedesktop.NetworkManager", "WirelessEnabled",
                          True)
        # Give the WiFi adapter some time to scan for APs. This is absolutely
        # the wrong way to do it, and the program should listen for
        # AccessPointAdded() signals, but it will do.
        time.sleep(10)

    accesspoints_paths_list = device.GetAccessPoints()

    # Identify our access point. We do this by comparing our desired SSID
    # to the SSID reported by the AP.
    our_ap_path = None
    for ap_path in accesspoints_paths_list:
        ap_props = dbus.Interface(
            bus.get_object("org.freedesktop.NetworkManager", ap_path),
            "org.freedesktop.DBus.Properties")
        ap_ssid = ap_props.Get("org.freedesktop.NetworkManager.AccessPoint",
                               "Ssid")
        # Returned SSID is a list of ASCII values. Let's convert it to a proper
        # string.
        str_ap_ssid = "".join(chr(i) for i in ap_ssid)
        print ap_path, ": SSID =", str_ap_ssid
        # Make a foreach something here
        if SSID_SLUG in str_ap_ssid:
            drones.append({'ap_path': ap_path, 'ssid': str_ap_ssid})
    return drones

def droneConnect(ssid):
    global device_path, connection_path, settings_path

    # Connect to the device's Wireless interface and obtain list of access
    # points.
    device = dbus.Interface(bus.get_object("org.freedesktop.NetworkManager",
                                           device_path),
                            "org.freedesktop.NetworkManager.Device.Wireless")
    accesspoints_paths_list = device.GetAccessPoints()

    # Identify our access point. We do this by comparing our desired SSID
    # to the SSID reported by the AP.
    our_ap_path = None
    for ap_path in accesspoints_paths_list:
        ap_props = dbus.Interface(
            bus.get_object("org.freedesktop.NetworkManager", ap_path),
            "org.freedesktop.DBus.Properties")
        ap_ssid = ap_props.Get("org.freedesktop.NetworkManager.AccessPoint",
                               "Ssid")
        # Returned SSID is a list of ASCII values. Let's convert it to a proper
        # string.
        str_ap_ssid = "".join(chr(i) for i in ap_ssid)
        print ap_path, ": SSID =", str_ap_ssid
        # Make a foreach something here
        if ssid == str_ap_ssid:
            our_ap_path = ap_path

    # At this point we have all the data we need. Let's prepare our connection
    # parameters so that we can tell the NetworkManager what is the passphrase.
    connection_params = {}

    # Establish the connection.
    settings_path, connection_path = manager.AddAndActivateConnection(
        connection_params, device_path, our_ap_path)
    print "settings_path =", settings_path
    print "connection_path =", connection_path

    # Wait until connection is established. This may take a few seconds.
    NM_ACTIVE_CONNECTION_STATE_ACTIVATED = 2
    print """Waiting for connection to reach """ \
          """NM_ACTIVE_CONNECTION_STATE_ACTIVATED state ..."""
    connection_props = dbus.Interface(
        bus.get_object("org.freedesktop.NetworkManager", connection_path),
        "org.freedesktop.DBus.Properties")
    state = 0
    while True:
        # Loop forever until desired state is detected.
        #
        # A timeout should be implemented here, otherwise the program will
        # get stuck if connection fails.
        #
        # IF PASSWORD IS BAD, NETWORK MANAGER WILL DISPLAY A QUERY DIALOG!
        # This is something that should be avoided, but I don't know how, yet.
        #
        # Also, if connection is disconnected at this point, the Get()
        # method will raise an org.freedesktop.DBus.Error.UnknownMethod
        # exception. This should also be anticipated.
        state = connection_props.Get(
            "org.freedesktop.NetworkManager.Connection.Active", "State")
        if state == NM_ACTIVE_CONNECTION_STATE_ACTIVATED:
            break
        time.sleep(0.001)
    print "Connection established!"

def disconnect():
    # Clean up: disconnect and delete connection settings. If program crashes
    # before this point is reached then connection settings will be stored
    # forever.
    # Some pre-init cleanup feature should be devised to deal with this problem,
    # but this is an issue for another topic.
    global device_path, connection_path, settings_path

    bus = dbus.SystemBus()
    # Obtain handles to manager objects.
    manager_bus_object = bus.get_object("org.freedesktop.NetworkManager",
                                        "/org/freedesktop/NetworkManager")
    manager = dbus.Interface(manager_bus_object,
                             "org.freedesktop.NetworkManager")
    manager_props = dbus.Interface(manager_bus_object,
                                   "org.freedesktop.DBus.Properties")

    manager.DeactivateConnection(connection_path)
    settings = dbus.Interface(
        bus.get_object("org.freedesktop.NetworkManager", settings_path),
        "org.freedesktop.NetworkManager.Settings.Connection")
    settings.Delete()

def scanDrones(buttonPressed):
    # Find new drones. For each drone found, check if the thing is in the list. If yes, make it bold faced
    drones = findNetworks()
    if len(drones) < 1:
        warning(window,"Make sure the batteries of the AR.Drones are connected.","No drones found!")
        return None

    store = builder.get_object("liststore1")
    for newDrone in drones:
        # loop through the ones in the storage and check if we have equal ssid
        found = False
        for drone in store:
            if newDrone['ssid'] == drone[0]:
                # SSID is equal. Do not add to list. Add that we found it though!
                found = True
                drone[4] = "Yes"
            #else:
                # drone[4] = "No"
                # TODO: Above line resets all status to No, except last. Not the idea
        if found == True:
            continue
        # New drone found. Add  to the list
        dialog = askDialog(window, "Please enter a prefix and trackable ID for the new drone.",newDrone['ssid']+" identification")
        if dialog != None:
            store.append([newDrone['ssid'], "", dialog[0], int(dialog[1]), "Yes", ""])

    optiTrackUpdate()

def warning(parent, message, title=''):
    dialogWindow = Gtk.MessageDialog(parent,
                      Gtk.DialogFlags.MODAL | Gtk.DialogFlags.DESTROY_WITH_PARENT,
                      Gtk.MessageType.QUESTION,
                      Gtk.ButtonsType.OK,
                      message)

    dialogWindow.set_title(title)
    dialogWindow.show_all()
    response = dialogWindow.run()
    dialogWindow.destroy()
    return response

def confirm(parent, message, title=''):
    dialogWindow = Gtk.MessageDialog(parent,
                      Gtk.DialogFlags.MODAL | Gtk.DialogFlags.DESTROY_WITH_PARENT,
                      Gtk.MessageType.QUESTION,
                      Gtk.ButtonsType.OK_CANCEL,
                      message)

    dialogWindow.set_title(title)
    dialogWindow.show_all()
    response = dialogWindow.run()
    dialogWindow.destroy()
    return response

def askIp(parent, message, title=''):
    dialogWindow = Gtk.MessageDialog(parent,
                          Gtk.DialogFlags.MODAL | Gtk.DialogFlags.DESTROY_WITH_PARENT,
                          Gtk.MessageType.QUESTION,
                          Gtk.ButtonsType.OK_CANCEL,
                          message)

    dialogWindow.set_title(title)
    dialogBox = dialogWindow.get_content_area()

    ipLabel = Gtk.Label()
    ipLabel.set_text("IP Address:")
    dialogBox.pack_start(ipLabel, False, False, 0)

    ipEntry = Gtk.Entry()
    ipEntry.set_size_request(250,0)
    dialogBox.pack_start(ipEntry, False, False, 0)

    dialogWindow.show_all()
    response = dialogWindow.run()
    ip = ipEntry.get_text() 
    dialogWindow.destroy()
    if (response == Gtk.ResponseType.OK):
        return ip
    else:
        return None


def askDialog(parent, message, title=''):
    dialogWindow = Gtk.MessageDialog(parent,
                          Gtk.DialogFlags.MODAL | Gtk.DialogFlags.DESTROY_WITH_PARENT,
                          Gtk.MessageType.QUESTION,
                          Gtk.ButtonsType.OK_CANCEL,
                          message)

    dialogWindow.set_title(title)
    dialogBox = dialogWindow.get_content_area()

    nameLabel = Gtk.Label()
    nameLabel.set_text("Prefix / name:")
    dialogBox.pack_start(nameLabel, False, False, 0)

    nameEntry = Gtk.Entry()
    nameEntry.set_size_request(250,0)
    dialogBox.pack_start(nameEntry, False, False, 0)

    trackableLabel = Gtk.Label()
    trackableLabel.set_text("Trackable ID (integer):")
    dialogBox.pack_start(trackableLabel, False, False, 0)

    trackableEntry = Gtk.Entry()
    trackableEntry.set_size_request(250,0)
    dialogBox.pack_start(trackableEntry, False, False, 0)

    dialogWindow.show_all()
    response = dialogWindow.run()
    name = nameEntry.get_text() 
    trackable = trackableEntry.get_text() 
    dialogWindow.destroy()
    if (response == Gtk.ResponseType.OK) and (isinstance(int(trackable), (int))):
        return [name, trackable]
    else:
        return None

def launchController(treeview, row, column):
    # This function should only run when connected to the dedicated network. This can be done through wifi and cable though, so do not do it automatically.
    # Check for active drones, otherwise give warning
    global controllerProcesses

    store = builder.get_object("liststore1")
    if store[row][1] == "":
        warning(window, "Please assign IP's first.","No IP address assigned to this drone.")
        return None

    if len(controllerProcesses) < 1:
    # It can be that the ssid is already active, give no warning then
        response = confirm(window,"Please make sure you are connected to the dedicated network before running a controller.")
        if response != Gtk.ResponseType.OK:
            return None

    if store[row][5] == "Yes":
        warning(window, "Controller already active.","A controller was already initiated for this drone.")
        return None
    
    store[row][5] = "Yes"
    #thread = popenAndCall(store[row][0], garbageCollection, ["rosrun", "ram", "controller.py", store[row][2], store[row][1]])
    th = controllerThread(store[row][0], garbageCollection, ["rosrun", "ram", "controller.py", store[row][2], store[row][1], str(int(connectors))])
    th.start()
    controllerProcesses.append(store[row][0])
    #subprocess.Popen(["rosrun", "ram", "controller.py", store[row][2], store[row][1]])
        
    # Launch actual controller

def garbageCollection(ssid):
    global controllerProcesses
    # Get list of processes and remove this one
    controllerProcesses.remove(ssid) 

    store = builder.get_object("liststore1")
    for drone in store:
        if drone[0] == ssid:
            drone[5] = "No"


def quitCallback(widget, event):
    global controllerProcesses, optiThread
    if len(controllerProcesses) > 0:
        warning(window, "Controllers active.","There are active controller processes. Please close them first.")
        return True

    try:
        optiThread.poll()
        if(optiThread.returncode == None):
            optiThread.terminate()
    except (NameError, AttributeError):
        pass

    try:
        joyThread.poll()
        if(joyThread.returncode == None):
            joyThread.terminate()
    except (NameError, AttributeError):
        pass

    Gtk.main_quit(widget, event)

def btnClear(buttonPressed):
    store = builder.get_object("liststore1")
    for (i, drone) in enumerate(store):
        if(drone[5] != "Yes"):
            store.remove(drone.iter)

def rightClick(treeview, event):
    if event.button == 3: # right click
        store = builder.get_object("liststore1")
        path, col, x, y = treeview.get_path_at_pos(int(event.x), int(event.y))
        ip = askIp(window, "Enter custom IP","")
        if ip != None:
            treeiter = store.get_iter(path)
            store.set_value(treeiter, 1, ip)
            store.set_value(treeiter, 4, "No")

def optiTrackUpdate():
    global optiThread, connectors
    try:
        optiThread.poll()
        if(optiThread.returncode == None):
            optiThread.terminate()
    except (NameError, AttributeError):
        pass

    # Write yaml
    rospack = rospkg.RosPack()
    package_location = rospack.get_path("ram")
    store = builder.get_object("liststore1")
    
    maxvalue = 0

    with open(package_location+'/launch/optitrack_instance.yaml', 'w') as outfile:
        outfile.write("rigid_bodies:\n")
        for drone in store:
            outfile.write("    '"+str(drone[3])+"':\n")
            outfile.write("        pose: "+drone[2]+"/unfiltered_pose\n")

            if drone[3] > maxvalue:
                maxvalue = drone[3]

        # ASSUMPTION: All mass connectors have subsequent trackable IDs. The first mass connector has is the highest ID in the drone list + 1.
        connector = 0
        while connector != connectors:
            connector += 1
            outfile.write("    '"+str(maxvalue + connector)+"':\n")
            outfile.write("        pose: massConnector"+str(connector)+"/unfiltered_pose\n")

    # run optitrack
    
    optiThread = subprocess.Popen(["roslaunch", "ram", "optitrack_instance.launch"])

def adjMassConnectors(adj):
    global connectors
    connectors = adj.get_value()
    optiTrackUpdate()

if __name__ == "__main__":

    m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
    try:
        code, msg, val = m.getSystemState('interface_instance')
    except socket.error:
        print "Please start roscore first."
        sys.exit()

    GObject.threads_init()

    # Enable joystick
    joyThread = subprocess.Popen(["roslaunch", "ram", "joystick_module.launch"])

    rospack = rospkg.RosPack()
    package_location = rospack.get_path("ram")
    gladefile = package_location+"/py/interface.glade"
    builder = Gtk.Builder()
    builder.add_from_file(gladefile)

    handlers = {
        "onDeleteWindow": quitCallback,
        "btnImport": importDrones,
        "btnScan": scanDrones,
        "btnExport": exportDrones,
        "btnAssign": assignIP,
        "launchController": launchController,
        "btnClear": btnClear,
        "rightClick": rightClick,
        "adjMassConnectors": adjMassConnectors
    }
    builder.connect_signals(handlers)

    bus = dbus.SystemBus()
    # Obtain handles to manager objects.
    manager_bus_object = bus.get_object("org.freedesktop.NetworkManager",
                                        "/org/freedesktop/NetworkManager")
    manager = dbus.Interface(manager_bus_object,
                             "org.freedesktop.NetworkManager")
    manager_props = dbus.Interface(manager_bus_object,
                                   "org.freedesktop.DBus.Properties")

    
    # Get path to the 'wlan0' device. If you're uncertain whether your WiFi
    # device is wlan0 or something else, you may utilize manager.GetDevices()
    # method to obtain a list of all devices, and then iterate over these
    # devices to check if DeviceType property equals NM_DEVICE_TYPE_WIFI (2).
    device_path = manager.GetDeviceByIpIface(wlan_interface)
    print "wlan path: ", device_path

    # Connect to the device's Wireless interface and obtain list of access
    # points.
    device = dbus.Interface(bus.get_object("org.freedesktop.NetworkManager",
                                           device_path),
                            "org.freedesktop.NetworkManager.Device.Wireless")

    window = builder.get_object("window1")
    window.show_all()
    Gtk.main()