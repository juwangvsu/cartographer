----12/2/21 python code to calculate odom from raw imu data turtlebot bag ---
assume orientation know from the odom topic.
	python odomimu2csv.py test3.csv
		convert imu and odom topic to csv
	test3.csv: 
		converted from topics from turtlebot3_3_gazebo.bag
	odomimutf_example.py

turtlebot3_3_gazebo.bag: when vehicle rotate-move, the x-acc and y-acc is not clear-cut
	iinitial pose: -90deg z-axis
	2993 - 3001 idle

	3001.496 -3002.3 x-acc + , x-vel ramp up 
	3001 - 3011 x-body -3 meters 
	3010.4 - 3011.3  x-acc -, x-vel dec down

	3011-3017 rotate z-axis +90 deg

	3018.606 - 319.2 x-acc +, x-vel ramp up
	3019-3034 x-body + 3 meters 
	3033.021 - 3034.5: x-acc -, x-vel dec down 

	3034-3041 rotate z-axis +100 deg

	3040.456 - 3041.21 x-acc - x-vel ramp up
	3044.1 - 3045	x-acc +, y-acc strong, x-vel down
	
----11/30/21 verify carto result with 'ground truth'------------------
data:
	turtlebot3_imuodompt2.bag

traj_turtlebot_imuodompt2.txt
	traj2csv.py
	traj_turtlebot3d_carto.csv
	traj calculated by carto, points no time stamp
		point in local map frame, so start at (0,0,0)
	rostopic echo /trajectory_node_list -n 1 
	from time 233 to 248
ground truth:
	odom2csv.py
	odom_turtlebot3d_carto.csv
	/odom topic in the bag file, published by gazebo, in global frame
	starting point is not (0,0,0)

Obs:
	carto traj floating even robot is static (at begining)
	carto traj seems scaled down, x 3 meters vs 4 meters gt.

TBD: test pure imu integral result.

------11/29/21 homepc cartographer turtlebot3 gazebo rosbag record----------------------
turtlebot3_slam/launch/turtlebot3_house_bringup.launch 
	topics: 
		/odom, /imu, /camera/depth/points, /camera/depth/image_raw, /camera/rgb/image_raw, 
		/camera/depth/camera_info, /camera/rgb/camera_info, /scan
	frame_id: odom,base_footprint, camera_depth_optical_frame,camera_depth_optical_frame, camera_depth_optical_frame, 

fixed urdf launch so imu publish with gravity. 
	- worked. issues solved:
	   -- no fixed joint in gazebo, workaround with revolute joint
	   -- camera_link inertial fixed. inertial required when change
		camera_joint to revolute, otherwise the link not work in 
		gazebo, and the camera plugin not work. if camera_link is
		fixed, the joint will not be recogized by gazebo, but the
		camera_link would work without inertial and camera plugin
		work, as in the stock turtlebot3_description setup.
	   -- bad joint/link/inertial also make robot not moving plugin 
		fail
	   -- compare turtlebot3_description/urdf/turtlebot3_waffle.urdf
		for detail
	   -- negative gravity vector:
		imu sensor pose <pose>0 0 0 3.14159 0 0</pose>, so the observed
		imu have gravity acc pointing down (negative). this set is
		in turtlebot3_description/urdf/turtlebot3_waffle.gazebo.xacro
record bag:
	roslaunch turtlebot3_slam/launch/turtlebot3_house_bringup.launch 
	rosbag record -O turtlebot3_imuodompt2.bag /scan /odom /imu /camera/rgb/camera_info /camera/rgb/image_raw/compressed /camera/depth/camera_info /camera/depth/image_raw /camera/depth/points

play recorded bag:
	roslaunch turtlebot3_slam/launch/turtlebot3_bringup_tf.launch
	rosbag play turtlebot3_imuodompt2.bag --clock

test with carto:
	roslaunch launch/launch/turtlebot3_3d.launch
	rosbag play turtlebot3_imuodompt2.bag --topics /imu /camera/depth/points /camera/depth/points:=/camera/depth/points2 /imu:=/mavros/imu/data --clock
	carto node work, result no good. pt2 seems at z-axis, rotate needed? 
	fixed:	create corresponding urdf for simplified turtlebot
		turtlebot3_3d.urdf camera_depth_optical_frame_joint
			  rpy="1.571 0. -1.571"
	pt2 dispaly slugish, point cloud too dense?
		play-pause bag file make cpu easy.
	carto trajectory good up to 16 sec. at 17 sec the pt2 change too much
		cause scan matching fail and result in bad traj.

videos:
	turtlebot3_gazebo_imupt2.mp4 -------- turtlebot3_house_bringup.launch
	turtlebot3_gazebo_imupt2_recordbag.mp4 -------- record turtlebot3_imuodompt2.bag
	turtlebot3_gazebo_imupt2_carto.mp4 -------- first carto test with turtlebot3_imuodompt2.bag 
 

see 9/4/2021 note

----------11/26/2021 b3 bag imu taint experiment  -----
setting TRAJECTORY_BUILDER_2D = {
  use_imu_data = false,
seems not affecting the carto result

taint imu effect limited, neg y,z of the imu data seem flip the resultant
trajectory by rotate x-axis 180 deg
adding offset to z or neg z only no effect.

----------11/26/2021 mavros short bag debugging -----
     then tinker the mavros_realsense_short.bag,
	transcode mavros_realsense_short.bag to have the same topics and
	frame_id as b3_transrecord.bag. mavros bag file contain /mavros/imu/data 	but frame_id is base_link; it also have depthimage: also imu z gravity
	must be negative.
	
	- roscore; rosparam set /use_sim_time true
	 -rosbag record -O mavrosshort_transrecord.bag /mavros/imu/data /camera/depth/points2	
        -roslaunch depth_image_proc/depth_image.launch
	 - python baglistener_mavrosshort.py
        -rosbag play mavros_realsense_short.bag /mavros/imu/data:=/tmpimu --clock
	test:
	-	camera_depth frame rotated so the pt looks normal
		p3at_3d_90deg.urdf
	- roslaunch launch/p3at_3d_90deg.launch
	- rosbag play mavrosshort_transrecord.bag --clock
	status:
		carto node seems received the imu and pt2 data
		but fail to calculate, another issue with clock
		within data? 

		or check if the range data related setting.
		or lua param about # of accumulated pt2 too high
			the b3_xxx bag pt2 msg only contain a small
			section, so lua param num_accumulated_range_data =160
			change to 1.
			this seems work. the carto node is calculating and
			publish map->odom tf, but still crash.
			

----------11/24-25/2021 demo cartograph 3d rosbag simplify -----
simpflied backpack_3d demo work:
	- b3_rerecord.bag is record from the b3_xxx bag file
	- only horizental laser and imu are recorded
	- launch file and lua file edited accordingly
	- work file:

steps:
	- roscore; rosparam set /use_sim_time true
	- rosbag record  -O b3_rerecord.bag /imu /horizontal_laser_3d
	- roslaunch launch/demo_backpack_3d_nobag.launch
	- rosbag play b3_rerecord.bag --clock

idea: convert b3_rerecord.bag to change frame_id field to test it with
	p3at_3d.launch
     then tinker the mavros_realsense_short.bag

status:
   failed try:
	bag.py to convert bagfile 80%, end_time not working
	carto node not working, warn msg seems indicate time stamp mismatch
	compare two lua files: p3at and backpack_3d
	problem: when write to bag file, the last chunk end_time is the current
	system time, which is probably a end-of-chunk msg appended at close()
	
	11/25 update: bag.py generated test.bag buggy. multiple places of inconsisitant time stamps.

   current try:
	try to transcode topics in a different way use a listener/publisher:
	new bag file step:	
	- roscore; rosparam set /use_sim_time true
	 -rosbag record -O b3_transrecord.bag /mavros/imu/data /camera/depth/points2	
	 - python baglistener.py
	 - rosbag play b3_rerecord.bag --clock

	new bag file test seems working:	
	- roslaunch launch/p3at_3d_b3bag.launch
	- rosbag play b3_transrecord.bag --clock
	- carto node working and rviz showing stuff. the map is not correct.
		- resolved: b3_transrecord.bag's /mavros/imu/data's frame_id
		incorrectly set to base_link, fixed baglistener.py so the imu
		frame_id=imu_link
		p3at_3d_b3bag.launch use p3at_3d_b3bag.lua
		p3at_3d.launch use p3at_3d.lua
			num_accumulated_range_data = 1 or 160 dep on bag files

TBD: 
     then tinker the mavros_realsense_short.bag,

----------11/23/2021 turtlebot  cartograph 2d rosbag  -----

simplified launch file for cartographer 2d + turlebot3 bagfile
	- /media/student/data6/cartographer$ roslaunch launch/turtlebot3_2d.launch
	- rosbag play turtlebot3_3_gazebo.bag --topics /imu /odom /scan --clock

status:
	seems work, the map is crooked. this is due to lidar_link joint
	in turtlebot3_simple.urdf's rpy not zero. fixed
	use_sim_time=true in launch file

rosbag re-record from bagfile:
	to preserve same time as old bag file, use_sim_time=true
	-roscore; rosparam set /use_sim_time true
	-rosbag record  -O turtlebot3_rerecord.bag /imu /odom /scan	
	-rosbag play turtlebot3_3_gazebo.bag --topics /imu /odom /scan --clock
	-verify timestamp and topics: rosbag info new.bag
	-verify turtlebot3_rerecord.bag with carto launch above

see video: cartographer_turtlebot_3.mp4
see 11/7 turtlebot carto notes

----------11/23/2021 p3at cartograph realsense bag  -----
cartographer on turtlebot3_gazebo.bag
	-roslaunch depth_image_proc/depth_image.launch
	-/media/student/data6/cartographer$ roslaunch launch/p3at_3d.launch
		configuration_files/p3at_3d.lua
	-rosbag play mavros_realsense_short.bag --clock

	note: /use_sim_time = true at launch file, rosbag must --clock to drive robot_state_publisher, if not set or false, robot_state_publisher will use system clock, it should still work
	
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
	roscore;rosparam set /use_sim_time true
	~/Documents/cartographer$rosbag play turtlebot3_3_gazebo.bag --topics /imu /odom /scan --clock
	/media/student/data6/cartographer$ roslaunch turtlebot3_slam/launch/turtlebot3_slam.launch slam_methods:=cartographer
	turtlebot3_slam/config/turtlebot3_lds_2d_gazebo.lua
	
	use odom result is good. not using odom result map has errors.
	work, see video cartographer_turtlebot.mp4
	
	note 11/23: if play all topics, the recorded tf allow robot model to show. if only play sensor data topics, carto node publish odom->base_footprint depending dep on /clock and sensor data timestamp. rosbag --clock publish /clock using time info in the bag file even if the bag file does not contain /clock data. it will use /imu time stamp for example.
		(1) /use_sim_time=true & --clock: all is good. 
		(2) /use_sim_time=true & no clock, no nothing. 
		(3) /use_sim_time=false & no clock, no robot visual, scan and map good, odom->base_footprint tf exist but xyz translation totally wrong.
		(4) /use_sim_time=false & --clock, no robot visual, scan and map good. odom->base_footprint tf exist but xyz translation totally wrong. 
		(5) see video cartographer_turtlebot_2.mp4

	tf files for the four cases:
		turtlebot3_slam/frames_turtlecarto_1/2/3/4.pdf

	when odom-> base_footprint tf is pulished by carto node, such as in case (1), this tf is the chaning robot pose. the robot visual is moving in rviz. if we force a static odom->base_footprint tf, the robot visual remain static in rviz

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

------11/27/21 homepc build  cartographer_ros from src----------------------
/media/student/data6/catkin_ws
using ninja tool
	catkin_make_isolated --install --use-ninja
	this will download both cartographer and cartographer_ros
	and build cartographer_ros

# To Build  Cartographer seperately (to access test)
cd /media/student/data6/catkin_ws/src/cartographer
mkdir build
cd build
cmake .. -G Ninja
ninja
	all the tests will be compiled under build/
	./cartographer.transform.transform_test

CTEST_OUTPUT_ON_FAILURE=1 ninja test
	this run all tests

#sudo ninja install

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
