Environment setup
======================
The environment this package should be used in, consists of multiple elements. The requirements for those elements are shortly mentioned. 

Computer
----------
All code is developed and tested under Ubuntu 12.04 running ROS Hydro. The computer should have a Wireless Interface to connect to the drones. In the optimal case, this wireless connection is only used for the configuration of the drones. After configuration the drones connect to a wireless accesspoint, to which the computer can be connected through a cable. 
> The connection from the computer to the accesspoint can also be wireless. In this way you introduce an additional bottleneck in your system however, since all data (including video data) goes through this connection. **NB** Adding drones while other drones are flying is impossible if you have just a wireless connection to the accesspoint. You will lose connection to the flying drones.

Wireless Action Point
------------------
A wireless action point should be available for the use in this setup. The connector should be configured correctly to use this action point (see the [connector](connector.md) page). The access point should supply an unprotected wireless network, preferably making use of the Wifi N-standard.

OptiTrack
----------
The OptiTrack should be installed and correctly calibrated. All drones should be introduced as "Rigid bodies" in the software supplied by Natural Point (Motive). In the streaming pane of the software, the IP address of the computer running the [mocap_optitrack][1] package should be configured. This is explained in detail on their Github page (section 2).


  [1]: http://wiki.ros.org/mocap_optitrack