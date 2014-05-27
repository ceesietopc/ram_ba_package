#!/bin/bash

# TODO
# - Test root requirement
# - Test if errors are gone for the service stopping
# - Put all lines in the file at once. Or at least using one telnet session. This is dirty.
# - Make more stable: timer / if statement around the dhclient statement


# Important! This script should be used with root permissions. Use "sudo bash initDrones.sh"
if [[ $EUID -ne 0 ]]; then
   echo "You must be root to do this." 1>&2
   exit 100
fi


# Restart networking tool to disable current connections
gksu service network-manager stop > /dev/null
wait
gksu service network-manager start > /dev/null
wait

# Get drones while network tool is still up
drones=()
while read -r network ; do
    drones+=(${network%%:*})
    echo "Drone found:"
    echo ${network%%:*}
done < <(nm-tool | grep ardrone)
wait

# disable networking tool
$(gksu service network-manager stop > /dev/null)
wait

droneip=10;
for drone in ${drones[@]}; do
   $(sudo iwconfig wlan0 essid $drone)
   wait
   $(sudo dhclient wlan0)
   wait
   echo "Connected to $drone. Starting telnet."
    # remove old file and write new file
    #$(( echo "rm /data/awifi.sh"; echo "echo -e \"killall udhcpd\nifconfig ath0 down\niwconfig ath0 mode managed essid RAM_drones\nifconfig ath0 192.168.1.$droneip netmask 255.255.255.0 up\" > /data/awifi.sh"; echo "chmod +x /data/awifi.sh"; echo "sh /data/awifi.sh" ) | telnet 192.168.1.1)
    wait
    echo "Finalized initialization of $drone. Drone can be reached at 192.168.1.$droneip."
    droneip=$((droneip+1))
    
done

wait
$(gksu service network-manager start)