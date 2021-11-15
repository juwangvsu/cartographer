#!/bin/bash
#source ~/.bashrc
echo 'publish hover cmd to /cmd_vel, for 10 hz continues cmd, we need gnome terminal so we can stop it if desired. the one shot hover does not need a terminal since it only publish one msg'
echo $1 $2 $3
#$1 gototag, or gototag2 
echo "host name: $ROS_HOSTNAME"
        #xterm -e "echo $1 $2;sleep 10"
if [ $1 = True ]
then
	 #rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}' 
	 gnome-terminal -x $SHELL -ic	"term_title irissetposition; rostopic pub -1 /mavros/setpoint_position/local geometry_msgs/PoseStamped '{pose: {position: {x: 0, y: 0, z: 2}, orientation: {x: 0, y: 0, z: 0}}}' ;bash"
else
	rostopic pub -r 3 /mavros/setpoint_position/local geometry_msgs/PoseStamped '{pose: {position: {x: 1.0, y: 0, z: 2}, orientation: {x: 30, y: 0, z: 0}}}'
	echo 'now call set_mode'
	rosservice call /mavros/set_mode '{custom_mode: OFFBOARD}'
	echo 'now call arming'
	rosservice call /mavros/cmd/arming '{value: true}'
	echo 'now keep sending position'
		# must publish faster than 2HZ to keep fail safe off, which
		# is necessary for offmod op, not sure this would work with
		# the original iris firmware (apm)
	for i in {1..10}
	do
		echo $i
		echo 'fly to position ', $i, ' ctl c to next position'
		posemsg="'{pose:{position:{x:1.0,y:0,z:2},orientation:{x:0,y:0,z:0}}}'"
		#echo $posemsg
		if [ $i = 1 ]
		then 
		rostopic pub -r 3 /mavros/setpoint_position/local geometry_msgs/PoseStamped '{pose: {position: {x: 1.0, y: 0, z: 2}, orientation: {x: 0, y: 0, z: 0}}}'
		fi
		if [ $i = 2 ]
		then 
		rostopic pub -r 3 /mavros/setpoint_position/local geometry_msgs/PoseStamped '{pose: {position: {x: 10.0, y: 0, z: 2}, orientation: {x: 0, y: 0, z: 90}}}'
		fi
		#rostopic pub -r 3 /mavros/setpoint_position/local geometry_msgs/PoseStamped $posemsg
	done	

fi
