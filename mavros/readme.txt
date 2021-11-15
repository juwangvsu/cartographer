-------------6/2/2019 retest mavros and solo ------------
see ~/bin/README.txt
	mavros 0.29.2 fcu_protocol must be v1.0 for solo
	use solo_apm_udp.launch

-------------5/30/2019 retest mavros and solo------------
roslaunch iris_apm_udp.launch
rosservice call /mavros/cmd/arming '{value: true}'
rosrun mavros mavcmd long 400 1 0 0 0 0 0 0
	this is the arming cmd. note long instead of int
	this require mavros_node running, otherwise error:
	service [/mavros/cmd/command] unavailable

this works on ub14, but not ub16. 
related note see readme file from the launch_uavcam.sh, help button.

debug progress:
	ub16, when launch iris_apm_udp.launch, it connect, but then 
	it complain lot of 
		unsolicited param value idx=366, not resetting retries count 3
	this does not happen at ub14	
	
-------------5/28/2019 offb_node created under new pkg------------

	the offb_node can be build with the modified the CMakeLists.txt
	but this cause problem, as ros will use the mavros pkg in turtlebot
	as the first choise, where we commented the portion to build
	mavros_node, thus rosrun mavros mavros_node can't find the executable.

	the solution is to put back CATKIN_IGNORE to not build mavros.

	a new package mavros_apps is created to host offb_node

-------------5/28/2019------------------------------------
	the mavros code here is old. so don't build and
	use the binary code here. the mavros installed
	under /opt/ros/... should be used. this mean that
	make sure the rospath and pythonpath are correct
	when calling mavros cmd etc.
	
	only launch file are usful.

	in particular, the local workspace should not contain
	executable stuff from this folder. if they exist
	and cause interference to mavros binary under /opt/ros,
	remove devel, build and make sure ignore file is put here
	and rebuild other turtlebot pkgs.

		also uav2cam.ui adding a few button to call rosservice directly for testing
			add sitl checkbox to launch individual sitl instance.

		also add  CATKIN_IGNORE on all ~/turtlebot/src/mavros/mavros subpkgs
			and rm devel, build, and catkin_make
			so mavsys will use the python library @ /opt/ros/...
		also add CATKIN_IGNORE on ~/turtlebot/src/turtlebot_apps/ to build
