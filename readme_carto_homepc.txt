
----------11/23/2021 p3at cartograph realsense bag  -----
cartographer on turtlebot3_gazebo.bag
	-roslaunch depth_image_proc/depth_image.launch
	-/media/student/data6/cartographer$ roslaunch launch/p3at_3d.launch
		configuration_files/p3at_3d.lua
	-rosbag play mavros_realsense_short.bag --clock

	note: /use_sim_time = true at launch file, rosbag must --clock to drive robot_state_publisher
	
	status:
		carto node seems taking input topic data, but is not working.
		robot model and tf ok, but map->base_link not working due to cartographer_node
		not working yet
		point cloud from depth_image_proc work, but matched pt2 (pub from carto node)
		nothing yet.

----------11/13/2021 realsense bag study -----
the realsense bag files (recorded by ros or realsense-viewer) only have depth image, not point cloud.
rs-convert convert the depth img to point cloud
rs-convert -d -l plyfile -i realsense_viewer_val.bag
	the tool however dont work with mavros_realsense_short.bag
	which is recorded using ros
meshlab plyfile_169793.55499999999302.ply

depth image can be converted to point cloud using:
	- depth_image_proc
	- roslaunch depth_image_proc/depth_image.launch
		the nodelet use depth/image_rect_raw, depth/camera_info topic, but did rostopic info
		will not show the topic being subscribed, per nodelet property
	- rosbag play mavros_realsense_short.bag
		pub: points
	- rviz, frameid, camera_depth_optical_frame

video:
	realsense_depth_image_points.mp4

-----------11/13/2021 two depth images from realsense bag files----
depth_realsense_viewer_val.txt
	rostopic echo -n 1 /device_0/sensor_0/Depth_0/image/data > depth_realsense_viewer_val.txt
depth_realsense_ros_val.txt
	rostopic echo -n 1 /camera/depth/image_rect_raw > depth_realsense_ros_val.txt

TBD:
need to verify that the depth image in both type of bag file are of the same
format. and find/impl a ros node to do real time conversion, sub to depth img
and pub pt2 topic

validation bag file: both recorded at home, at a similar position
	realsense_ros_val.bag
	realsense_viewer_val.bag

--------11/13/2021 realsense rosbag record------------
roslaunch realsense2_camera rs_camera.launch enable_gyro:=true enable_accel:=true unite_imu_method:=linear_interpolation
        enable_color:=false enable_depth:=false cut cpu core to 60%
        enable_pointcloud:=true pointcloud_texture_stream:=RS2_STREAM_ANY
        pose not supported for l515
rosbag record /camera/motion_module/parameter_descriptions /camera/l500_depth_sensor/parameter_descriptions /camera/imu /camera/gyro/imu_info /camera/extrinsics/depth_to_color /camera/depth/image_rect_raw /camera/depth/camera_info /camera/color/image_raw/compressed /camera/color/image_raw/compressed/parameter_descriptions /camera/color/image_raw /camera/color/camera_info /camera/accel/imu_info

see readme_nvidia_nano.txt 9/8/21 note

----------11/12/2021 cartographer bag study -----
3d:
	/horizontal_laser_3d
	/vertical_laser_3d

	pub:
	/scan_matched_points2
	/submap_list
		each trajectory point have one associated submap, 
		labeled: trajid, submap index id, version #: 0,0,320

to obtain a submap:
rosservice call /submap_query "trajectory_id: 0
submap_index: 0"  > submap_0.0.txt

----------11/8/2021 p3at gearup to collect bag data file -----
	hardware: px4#1 (rover), px4#2 (imu src), pi4b, realsense sensor
	px4#1 is customized to drive wheel. not able to set airframe, so no data
	px4#2 is to pump high res imu data to pi, 50 hz, use usb port. telem 
		port too slow (8 hz)
	pi4: Documents/cartographer/mavros/launch/, roslaunch iris_px4_ttyACM0.launch 
	pi4: set /etc/hosts pi4 ip address based on current ip or just localhost

----------11/11/2021 robot_localization, p3at bag file -----
	edit params/ekf_p3at.yaml
	/robot_localization/launch$ roslaunch launch/ekf_p3at.launch
	rosbag play mavros_realsense_long.bag 
	cartographer/robot_localization$ roslaunch launch/static_transforms_p3at.launch
	make sure /use_sim_time is not set or false


----------11/7/2021 hdl_400.bag, turtlebot bag, realsense data repo test -----
robot_localization pkg first test with realsense 
	edit params/ekf_template.yaml
	/robot_localization/launch$ roslaunch ekf_turtlebot.launch
	rosbag play turtlebot3_3_gazebo.bag
	cartographer/robot_localization$ roslaunch launch/static_transforms.launch
	make sure /use_sim_time is not set or false
		if set to true, rosbag play --clock, this will publish /clock based on rosbag file to drive other nodes.
	result is bad
	use imu data only
	see video robot_localization_turtlebot.mp4
	
cartographer on hdl_400.bag
	3D lidar scan
	imu data	

cartographer on turtlebot3_gazebo.bag
	~/Documents/cartographer$ rosbag play turtlebot3_3_gazebo.bag
	/media/student/data6/cartographer$ roslaunch turtlebot3_slam/launch/turtlebot3_slam.launch slam_methods:=cartographer
	turtlebot3_slam/config/turtlebot3_lds_2d_gazebo.lua
	use odom result is good. not using odom result map has errors.
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
