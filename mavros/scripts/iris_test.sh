# 9/28/18 editing: this is px4 + mavros demo
gnome-terminal -x $SHELL -ic "cd ~/Downloads/px4-1.5.5/Firmware; make posix_sitl_default gazebo" 
gnome-terminal -x $SHELL -ic "cd ~/turtlebot/src/mavros/mavros/launch; roslaunch iris_apm_udp.launch"
echo "wait about 15 secs for gazebo to be up"
sleep 15
rosservice call /mavros/cmd/arming '{value: true}'
rosrun mavros mavcmd takeoffcur 0  0 5 #this must after arming
sleep 5
rosrun mavros mavcmd landcur 0  0 
