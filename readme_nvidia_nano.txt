--- sticky start ngrok -------------
./ngrok start nano1

https://github.com/jetsonhacks/installRealSenseSDK.git

juwangvsu github:
git config --global credential.helper cache
Goto a repo, Git pull
Git personal access token: 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

-----ROS_DEBUG example ----------------------------------
rosservice call /turtlebot3_slam_gmapping/set_logger_level "logger: 'ros.gmapping'
level: 'debug'" 

-------11/10/2021 mavros plugin loaded for local_position ----
Plugin home_position loaded
Plugin imu loaded
Plugin local_position loaded

mavros topics related to pose, odom, velocity...
/mavros/imu/data
/mavros/local_position/odom --- xy 0, z changing, twist seems live data
/mavros/local_position/pose---------- almost identity to odom
/mavros/local_position/velocity_body --- almost identity to odom velecity
/mavros/local_position/velocity_local --- different then above, maybe transformed?

-----11/9/2021 realsense bag recording nano1 ---
files:
	rosbag_realsense.sh
	rosbag_realsense_110921-2.bag

topics:frame_id
	/camera/imu 		camera_imu_optical_frame			
	/camera/depth/color/points  camera_depth_optical_frame
video:
	realsense_rosbag_nano1.mp4
-----11/9/2021 mavros realsense bag recording nano1 ---
---
header: 
  seq: 2458
  stamp: 
    secs: 1636484484
    nsecs: 303363777
  frame_id: ''
attitude_status_flag: True
velocity_horiz_status_flag: False
velocity_vert_status_flag: True
pos_horiz_rel_status_flag: False
pos_horiz_abs_status_flag: False
pos_vert_abs_status_flag: True
pos_vert_agl_status_flag: False
const_pos_mode_status_flag: True
pred_pos_horiz_rel_status_flag: False
pred_pos_horiz_abs_status_flag: False
gps_glitch_status_flag: False
accel_error_status_flag: False


-----9/13/21 robot_pose_ekf robot_localization----------------
apt install ros-melodic-robot-localization
apt install ros-melodic-robot-pose-ekf

these two pkgs estimate pose from imu and other sensors.
test bag:
	l515_imu.bag (hokuyo_node)

-----9/12/21 turtlebot3 bringup vs gazebo odom base_footprint---
physical robot:
	turtlebot3_bringup turtlebot3_robot.launch
	controller: pi + opencr
		pi handle ros stuff, opencr low level stuff, include
		imu sensors.
	driver: pi <-> opencr using rosserial_python
		published topics: odom->base_footprint tf and more
		opencr impl rosserial, which pack all topics in serial msgs.
		pi runs rosserial_python client node and publish
		all topics in ros space.
		opencr firmware: arduino code,
			https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/turtlebot3/examples/turtlebot3_burger/turtlebot3_core/turtlebot3_core.ino
	        sprintf(odom_header_frame_id, "odom");
	        sprintf(odom_child_frame_id, "base_footprint");  



gazebo robot:
	turtlebot3_gazebo turtlebot3_house.launch
	gazebo plugins publish all required topics.

-----9/11/21 gmapping hokuyo lidar fake odom base_footprint---

current p3at_slam.launch:
	since no robot movement, map produced by gmapping is 'static'
	to make robot move, fake tf broadcast
		odom->base_footprint
		using turtlebotsim.launch
		this seems work yet, map is being updated as turtlesim moved by teleop
		in a gazebo+slam case, gazebo publish odom->base_footprint,
		slam node publish map-> odom
		robot_state_publisher publish base_footprint->rest of stuff
			see frames.pdf, frames_gz.pdf

gmapping slam node
	sub to /tf, /scan to build map
		it update map when see enough movement (from tf?)
	it publish /tf: map->odom

----- 9/12/21 tf_static.launch in p3at_slam.launch ---------
	tf_static.launch fake odom-base_footprint tf
        the actual tf reflect the movement of the robot.
        if gazebo simulation, then gazebo will publish this
        if actual robot, the robot driver will do this
        only include this for testing and see the lidar data.

-----9/8/21 test slam with hokuyo lidar ------------------
70% working:
	rosparam set /use_sim_time false
		otherwise joint_state_publisher not publishing
	joint_state_publisher_gui to publish /joint_states
	joint_states drive robot_state_publisher to pub /tf
	tf_static.launch to pub tf: odom->base_footprint
	/tf allow rviz to show stuff
	/tf allow gmapping to work

p3at_slam.launch modified from turtlebot3_slam.launch
robot_state_publisher need /joint_state to publish tf
	which is published by gazebo if simulation.
	it publish static tf.
	wheel_left_link not static, 30 hz, from /joint_states

/joint_states: (pub by gazebo)
name: [wheel_right_joint, wheel_left_joint]
position: [0.0031169588287500716, -0.001259621301003655]


test tf static:
rosrun  tf static_transform_blisher 0 0 0 0 0 0 /odom /base_footprint 100

-----9/8/21 realsense_ros test L515 lidar camera imu-------------------------
git pull new version if launch fail:
	~/catkin_ws/src/realsense-ros
	roslaunch realsense2_camera rs_camera.launch enable_gyro:=true enable_accel:=true unite_imu_method:=linear_interpolation
	enable_color:=false enable_depth:=false cut cpu core to 60% 
	enable_pointcloud:=true pointcloud_texture_stream:=RS2_STREAM_ANY
	pose not supported for l515
	
Topics published:
	rgb, depth, /camera/accel/sample, gyro/sample
	no imu by default
	both gyro and accel are of type Imu, but each only valid on gyro or accel part.
	unite_imu_method:=  combine accel and gyro, new topic /camera/imu
	l515_imu.bag
		
cpu % 
	130 image subscribed
	164   compressed image
	190	theora image
	106   no subscriber
 
----- 9/7/21 hokuyo_node lidar tested with nano1 --------------------
ros-melodice-driver-base
catkin_ws/src/hokuyo_node
rosrun hokuyo_node hokuyo_node
5v power supply direct, usb not for power.

------9/5/21 homepc + nano1 turtlebot3 + slam + cartographer ----------------
two machine testing to avoid wired networking problem:
        homepc (asus1):
                ROS_MASTER_URI=http://asus1:11311
                asus1 must be the actual ip, not 127.0.0.1
        nano1:
                ROS_MASTER_URI=http://asus1:11311
                ROS_HOSTNAME=nano1
        asus1, nano1 must be actual ip pingable at both machine.
:~/catkin_ws/src/frontier_exploration$ 

to use frontier_exploration:
	launch gazebo and slam cartographer
	roslaunch exploration_server exploration.launch 
	in rviz add Marker

------9/4/21 homepc nano1  cartographer turtlebot3 gazebo----------------------
roslaunch turtlebot3_gazebo ...
roslaunch turtlebot3_slam turtlebot_slam.launch slam_methods:=cartographer
        default cartographer launch file setup for real robot
        crash if using gazebo due to a gazebo imu bug.
        edit turtlebot_slam.launch
                <arg name="configuration_basename" default="turtlebot3_lds_2d_gazebo.lua"/>

2d case work. 3d case to be tested.

---------9/1/21 turtlebot3 stacks------------------
gazebo turtlebot3
        pub topics:
                odom, imu, scan, camera rgbd, pointcloud2

amcl node:
        pub topics:
                amcl_pose,/particlecloud
        sub topic:
                scan, tf, initialpose
turtlebot3_navigation.launch
        bringup, movebase, tf (state publisher), amcl, rviz
        mapserver (use a prior map file)

turtlebot3_slam.launch
        slam (gmapping), tf (state publisher), rviz
        note 2d goal not work, must launch move_base seperately.

move_base.launch
        move base node (dwa planner)

gazebo plugins:
        libgazebo_ros_diff_drive.so ----- cmd_vel odom
        libgazebo_ros_imu.so        ----- imu
        libgazebo_ros_laser.so      ----- scan
        libgazebo_ros_openni_kinect.so--- rgb/image_raw depth/image_raw depth/points

----------8/29/21 ros turtlebot3 installed ------------
gazebo work 70%, turtlebot3_gazebo plugin not work, no lidar data
fixed:
	apt install ros-melodic-gazebo-plugins ros-melodic-dwa-local-planner ros-melodic-gmapping ros-melodic-image-transport-plugins
ros-melodic-cartographer-ros

turtlebot3_slam.launch still not work, .
	lidar scan not following robot rotation movement
		this is coz lidar 360 deg

turtlebot3_cartograph
	tf static transform missing , eg, imu_link 
		such frames exist. 
	to test on newpcamd

------------8/23/21 debugging realsense, ros_yolo ----------
pyrealsense2 old version was not cleaned, cause "no device"
	work now
fix: make sure to remove old version.
	install with make install of the new librealsense 
	installed directory some times wrong place. to be fully understood.
	/usr/local/lib/py..3.6/
	/usr/lib/pyt..3.6/
	current library location:
		/usr/lib/python3.6/pyrealsense2.cpython-36m-aarch64-linux-gnu.so
	cd build
	cmake ../ -DBUILD_PYTHON_BINDINGS:bool=true
	make -j4
	sudo make install
	
	power is not issue here. 

ros_yolo/
	multiprocessing not work with nano, due to share memeory issue
	time sync still needed. manuel fine tuning.

ros/
	can install from  nvidia repo
	
------9/23/21 power voltage testing ------------------
load: wifi + realsense + usbhub (externally powered)

weak 12v + 5v converter not enough

lipo 3s + 5v converter 
	- good at boot
	- realsense_viewer + smoke demo cause power warning.
---------------8/22/21 testing realsense  -------------------------------
not working. 
	--- librealsense version mismatch? on nano 2.31, on pi 2.4

student@nano1:~/librealsense$ git log
commit bdce5a4ebbd716e9d7372ad8831a929bccccd0b2 (HEAD, tag: v2.31.0)
Merge: b60c383f0 3b3c64c24
Author: Sergey Dorodnicov <sergey.dorodnicov@intel.com>
Date:   Tue Dec 10 12:45:38 2019 +0200

    Merge pull request #5415 from radfordi/map-preservation

git checkout 2.48.0 version and rebuild from source
	realsense-viewer work    

pyrealsense2 still not work
python3 opencv_view_examples
no device found. yet rs_hello.. work

---------------8/22/21 testing python3 torch ~/object-detection -------------------------------

when python3, import numpy or torch, crash.
this is fixed by pip3 install numpy==1.19.4
	possible explaination is that some shared library was messed up somewhere. by reinstall numpy pkg, it uninstall some pkg and reinstall some corrrect
pkg.

student@nano1:~/NVIDIA_CUDA-10.2_Samples/2_Graphics/volumeRender$ python3
Python 3.6.9 (default, Jan 26 2021, 15:33:00) 
[GCC 8.4.0] on linux
Type "help", "copyright", "credits" or "license" for more information.
>>> import numpy
Illegal instruction (core dumped)

after fix the crash problem: 
python3 test_detection.py 
detect time:  1.0446128845214844
     _  class_id  confidence  x1   y1   x2   y2 class_name         label
0  0.0      18.0    0.872556  82  103  178  277        dog     dog: 0.87
1  0.0       1.0    0.659277   0   40   99  264     person  person: 0.65
detect time:  1.2126429080963135
       center_x    center_y           w  ...   y2  class_name      label
335  105.569516  181.654093  121.527615  ...  270         dog  dog: 0.31

[1 rows x 12 columns]

---------------8/22/21 testing examples  -------------------------------
cuda examples:
/usr/local/cuda/
https://www.youtube.com/watch?v=KROP46Wte4Q

---------------8/22/21 testing webcam -------------------------------
cheese and vlcplayer not work, core dump
ros uvc_camera works
gst works

https://forums.developer.nvidia.com/t/jetson-nano-faq/82953#5390621

test logitec webcam:
student@nano1:~$ v4l2-ctl -d /dev/video0 --list-formats-ext
lot of modes supported

student@nano1:~$ v4l2-ctl -d /dev/video1 --list-formats-ext
ioctl: VIDIOC_ENUM_FMT
	Index       : 0
	Type        : Video Capture
	Pixel Format: ''
	Name        : 00000032-0002-0010-8000-00aa003
		Size: Discrete 340x340
			Interval: Discrete 0.033s (30.000 fps)
gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! nvvidconv ! 'video/x-raw(memory:NVMM),format=NV12' ! nvoverlaysink
