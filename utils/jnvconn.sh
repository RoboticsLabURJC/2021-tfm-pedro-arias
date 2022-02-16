#!/bin/bash

if [ $# -eq 1 ]; then
	master=$1
else
	master="10.42.0.1"
fi

ip="sed -E 's/\.[0-9]+$/.1/;t;d' <<< $master"


export ROS_MASTER_URI="http://$master:11311/"
export ROS_IP="$ip"

command sshpass -p nvidia ssh nvidia@$master
