#!/usr/bin/python

# This script shows how to connect to a WPA protected WiFi network
# by communicating through D-Bus to NetworkManager 0.9.
#
# Reference URLs:
# http://projects.gnome.org/NetworkManager/developers/
# http://projects.gnome.org/NetworkManager/developers/settings-spec-08.html

import dbus
import time
import telnetlib
import json

SSID = "RAM_drones"
SSID_SLUG = "ardrone"
device_path = None
connection_path = None
settings_path = None

def findNetworks():
    drones = []
    global device_path
    bus = dbus.SystemBus()
    # Obtain handles to manager objects.
    manager_bus_object = bus.get_object("org.freedesktop.NetworkManager",
                                        "/org/freedesktop/NetworkManager")
    manager = dbus.Interface(manager_bus_object,
                             "org.freedesktop.NetworkManager")
    manager_props = dbus.Interface(manager_bus_object,
                                   "org.freedesktop.DBus.Properties")

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

    # Get path to the 'wlan0' device. If you're uncertain whether your WiFi
    # device is wlan0 or something else, you may utilize manager.GetDevices()
    # method to obtain a list of all devices, and then iterate over these
    # devices to check if DeviceType property equals NM_DEVICE_TYPE_WIFI (2).
    device_path = manager.GetDeviceByIpIface("wlan0")
    print "wlan0 path: ", device_path

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
        if SSID_SLUG in str_ap_ssid:
            drones.append({'ap_path': ap_path, 'ssid': str_ap_ssid})
    return drones

def droneConnect(ap_path):
    global device_path, connection_path, settings_path

    bus = dbus.SystemBus()
    # Obtain handles to manager objects.
    manager_bus_object = bus.get_object("org.freedesktop.NetworkManager",
                                        "/org/freedesktop/NetworkManager")
    manager = dbus.Interface(manager_bus_object,
                             "org.freedesktop.NetworkManager")
    manager_props = dbus.Interface(manager_bus_object,
                                   "org.freedesktop.DBus.Properties")

    # At this point we have all the data we need. Let's prepare our connection
    # parameters so that we can tell the NetworkManager what is the passphrase.
    connection_params = {}

    # Establish the connection.
    settings_path, connection_path = manager.AddAndActivateConnection(
        connection_params, device_path, ap_path)
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

class GUI:
    def __init__(self):

        #Set the Glad
        gladefile = "interface.glade"
        builder = gtk.Builder()
        builder.add_from_file(gladefile)
        builder.connect_signals(self)

        #self.window = self.wTree.get_widget("MainWindow")
        #if (self.window):
        #    self.window.connect("destroy", gtk.main_quit)


if __name__ == "__main__":
    drones = findNetworks()
    if len(drones) < 1:
        # NO DRONES, too bad
        print("NO DRONES")
    else:
        ip = 10;
        for (i, drone) in enumerate(drones):
            droneConnect(drone['ap_path'])
            drone['ip'] = "192.168.1."+str(ip);
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
            tn.write("echo \"ifconfig ath0 192.168.1."+str(ip)+" netmask 255.255.255.0 up\" >> /data/wifi.sh\n")
            tn.read_some()
            tn.write("chmod +x /data/wifi.sh\n");
            tn.read_some()
            #tn.write("sh /data/wifi.sh\n")
            # IMPORTANT At this point, we lose connectivity
            tn.write("exit\n")

            disconnect()

            ip+=1
        with open('drones.json','w') as outfile:
            json.dump(drones,outfile)

    exit(0)