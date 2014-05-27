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
$("service network-manager stop")
wait
$("service network-manager start")
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
$(service network-manager stop)
wait

droneip=10;
for drone in ${drones[@]}; do
   $(sudo iwconfig wlan0 essid $drone)
   wait
   $(sudo dhclient wlan0)
   wait
   echo "Connected to $drone. Starting telnet."
    # remove old file
    $(echo "rm /data/awifi.sh" | telnet 192.168.1.1)
    wait
    # write automated wifi file
    $(echo "echo \"killall udhcpd\" > /data/awifi.sh" | telnet 192.168.1.1)
    wait
    $(echo "echo \"ifconfig ath0 down\" >> /data/awifi.sh" | telnet 192.168.1.1)
    wait
    $(echo "echo \"iwconfig ath0 mode managed essid RAM_drones\" >> /data/awifi.sh" | telnet 192.168.1.1)
    wait
    $(echo "echo \"ifconfig ath0 192.168.1.$droneip netmask 255.255.255.0 up\" >> /data/awifi.sh" | telnet 192.168.1.1)
    wait
    $(echo "chmod +x /data/awifi.sh" | telnet 192.168.1.1)
    wait
    $(echo "sh /data/awifi.sh" | telnet 192.168.1.1)
    wait
    droneip=$((droneip+1))
    echo "Finalized initialization of $drone. Drone can be reached at 192.168.1.$droneip."
done

wait
$(service network-manager start)