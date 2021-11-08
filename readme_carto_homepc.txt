----------11/7/2021 hdl_400.bag, realsense data repo test -----
robot_localization pkg first test with realsense 
	/robot_localization/launch$ roslaunch ekf_turtlebot.launch
	rosbag play turtlebot3_3_gazebo.bag
	cartographer/robot_localization$ roslaunch launch/static_transforms.launch
	make sure /use_sim_time is not set or false
	result is bad
	use imu data only
	
cartographer on hdl_400.bag
	3D lidar scan
	imu data	

cartographer on turtlebot3_gazebo.bag
	~/Documents/cartographer$ rosbag play turtlebot3_3_gazebo.bag
	/media/student/data6/cartographer$ roslaunch turtlebot3_slam/launch/turtlebot3_slam.launch slam_methods:=cartographer
	work, see video cartographer_turtlebot.mp4
 
----------11/4/2021 realsense data repo test -----
hptitan: ~/Documents/cartographer/realsense-data

topics:
/device_0/sensor_2/Gyro_0/imu/data
/device_0/sensor_2/Accel_0/imu/data
/device_0/sensor_1/Color_0/image/data
/device_0/sensor_0/Depth_0/image/data

-----------cartographer demo info ---------------
b3...bag:
	Node [/play_1630984505180292361]
Publications: 
 * /clock [rosgraph_msgs/Clock]
 * /horizontal_laser_3d [sensor_msgs/PointCloud2]
 * /imu [sensor_msgs/Imu]
 * /rosout [rosgraph_msgs/Log]
 * /vertical_laser_3d [sensor_msgs/PointCloud2]

2d bag file
--------------------------------------------------------------------------------
Node [/play_1630984613604725760]
Publications: 
 * /clock [rosgraph_msgs/Clock]
 * /horizontal_laser_2d [sensor_msgs/MultiEchoLaserScan]
 * /imu [sensor_msgs/Imu]
 * /rosout [rosgraph_msgs/Log]
 * /vertical_laser_2d [sensor_msgs/MultiEchoLaserScan]

student@asus1:~/Documents/cartographer$ rostopic list
/clicked_point
/clock
/constraint_list
/horizontal_laser_2d
/imu
/initialpose
/joint_states
/landmark_poses_list
/map
/move_base_simple/goal
/rosout
/rosout_agg
/scan_matched_points2
/submap_list
/tf
/tf_static
/trajectory_node_list
/vertical_laser_2d

rosnode list
/cartographer_node
/cartographer_occupancy_grid_node
/playbag
/robot_state_publisher
/rosout
/rviz

submap
    trajectory_id: 0
    submap_index: 43
    submap_version: 14
    pose: 
      position: 
        x: -42.7713787509
        y: -56.691738426
        z: 0.0
      orientation: 
        x: 0.0
        y: 0.0
        z: -0.0292970551848
        w: 0.999570749151

------9/5/21 homepc + nano1 turtlebot3 + slam + cartographer ----------------
two machine testing to avoid wired networking problem:
	homepc (asus1):
		ROS_MASTER_URI=http://asus1:11311
		asus1 must be the actual ip, not 127.0.0.1
	nano1:
		ROS_MASTER_URI=http://asus1:11311
		ROS_HOSTNAME=nano1
	asus1, nano1 must be actual ip pingable at both machine.	

------9/4/21 homepc  cartographer turtlebot3 gazebo----------------------
roslaunch turtlebot3_gazebo turtlebot3_house.launch
roslaunch turtlebot3_slam turtlebot_slam.launch slam_methods:=cartographer
	call turtlebot_bringup turtlebot_remote.launch
		load urdf, robot_state_publisher, tf
	call turtlebot3_cartographer.launch
		 move_base.launcha
	 	 cartographer_node
		 cartographer_occupancy_grid_node
		 flat_world_imu_node
			/imu -> /flat_imu
	default cartographer launch file setup for real robot
	crash if using gazebo due to a gazebo imu bug.
	edit turtlebot_slam.launch 
		<arg name="configuration_basename" default="turtlebot3_lds_2d_gazebo.lua"/>

cartographer_node sub:
	 * /flat_imu [sensor_msgs/Imu]
	 * /odom [nav_msgs/Odometry]
	 * /scan
	pub:
	  /scan_matched_points2
	 * /submap_list [cartographer_ros_msgs/SubmapList]
	 /trajectory_node_list
	config file:
		turtlebot3_slam/config/turtlebot3_lds_2d.lua
		laser num, map_frame, odom_frame, use_odom,...

flat_world_imu_node: this node from turtlebot3i_slam pkg
	sub: /imu
	pub: /flat_imu

frontier_explore build from src catkin_ws/src
	explore_client no longer exist, which is used by turtlebot3_frontier_exploration.launch.

------9/2/21 homepc install cartographer ----------------------

install:

update apt key for ros, otherwise cartographer-rviz pkg not found
        sudo apt-key list #confirm ros key expired
        sudo apt-key del "C1CF 6E31 E6BA DE88 68B1  72B4 F42E D6FB AB17 C654"
        curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt install ros-melodic-cartographer ros-melodic-cartographer-ros ros-melodic-cartographer-ros-msgs ros-melodic-cartographer-rviz
cd /media/student/unity; mkdir cartographer;


dataset:

wget  https://storage.googleapis.com/cartographer-public-data/bags/backpack_2d/cartographer_paper_deutsches_museum.bag
wget https://storage.googleapis.com/cartographer-public-data/bags/backpack_3d/with_intensities/b3-2016-04-05-14-14-00.bag
cd Documents; ln -sn /media/student/unity/cartographer

test:

roslaunch cartographer_ros demo_backpack_2d.launch bag_filename:=${HOME}/Documents/cartographer/cartographer_paper_deutsches_museum.bag
roslaunch cartographer_ros demo_backpack_3d.launch bag_filename:=${HOME}/Documents/cartographer/b3-2016-04-05-14-14-00.bag

hdl_400.bag
	3d lidar,bag file from hdl_graph_slam/

get map:

roslaunch cartographer_ros offline_backpack_3d.launch bag_filenames:=${HOME}/Documents/cartographer/b3-2016-04-05-14-14-00.bag


get 2d map from topic /map
rosrun map_server map_saver 
eog map.pgm
