--------sticky --------------------------------------

build carto_ros:
	/media/student/data6/catkin_ws$ catkin_make_isolated --install --use-ninja
ref:
	"readme pointcloud ndt icp slam cartographer code "
	readme_carto_debug.txt
	readme_carto_rosbuild.txt
	readme_cartographer.txt
	readme_airsim.txt
carto 2.0.0:
        export ROS_PACKAGE_PATH=
        source /media/student/data6/catkin_ws/devel_isolated/setup.bash

---------4/15/22 debug mavros bag dataset ------------------
test:
	lua: mode=4 (ndt), savepcdflag=1
	cd ~/Documents/cartographer/
	roslaunch launch/p3at_3d_90deg.launch cartover:=2.0.0
	rosbag play mavrosshort_transrecord_2.bag --clock
	rosservice call /trajectory_query "trajectory_id: 0" > test/traj_0.txt
	
test data:
	cp ../test/* test_ceres_pcd/3-15-22-mavrosshort/
		traj_0.0.txt
		scanmatch_log.txt
		
status:
	match bad around:
	scanmatch_log.txt: line 10: 
		scan_hrt_8.pcd, -3.344, -0.239725, -0.0666702,
				-2.69143, 0.266681, 0.0305569
	the init x,y,z is not necessary the previous scan match pose,
	an expolation seems contribute to this. check imu data

	further evidence:
	according to traj_0.0.txt
	we have a jump at 
	       secs: 1636502968
	       nsecs: 209133900
	 from
	 x: -0.085 y: 0.0  z: 0.08 
	 to
	 x: -2.69 y: 0.26  z: 0.03
	scan_hrt_7.pcd, scan_hrt_8.pcd,
	checking the carto output. the init pose at scan_hrt_8.pcd is wrong, it is suppose to be close to the last traj estimation. further debug.
	see readme_carto_debug.txt



----------3/14/22 b3 bag file pt2 frame filter -------------
baglistener_filterpc2_b3.py
	similar to baglistener_filterpc2.py
	see 1/1/22 note
—------- 3/2/22 turtlebot3_imuodompt2_4.bag wide horizontal_fov ------------------
	2.6878, about 155 deg
	320x120, 20hz
	ndt result very good

--------3/1/2022 wangtest_main.cc ---------
this is a replic of icp_example.cc but build in cartograph space
run a scan match methods on testing pcd files. 
	mode: icp, ceres, ndt
build:
	cartograph build (standalone or via ros catkin)
run:
	catkin_ws/src/cartographer/build$ ./cartographer_wangtest 
input:
	testcfg.yaml
src:
	catkin_ws/src/cartographer/cartographer/io/

—------- 2/25/22 matlab code to extract pc edges ------------------
goal:
	icp match result might improve when only edge points are used.
	about 87.17 sec, scan_hrt_37.pcd vs submap_test_h_36.pcd

added to cartographer repo
	works at pc and linux,
	  input: test/turtlebot3_imuodompt2_3/map*.pcd
	  output:			edge_fastpcd_0.1/map*.pcd
	matlab/Point-clou.../xyz2depth.m
			edge_extrac_neighbor_search_pcd_fast.m


—------- 2/22/22 icp_example new mode ------------------
	ms -2: read both pcd file names and init pose from testcfg.yaml
		
	~/Documents/hdl_graph_slam/ndt/build/icp_example -pst=1 -rsl=0.02 -md=ndt -ms=-2
	~/Documents/cartographer/test_ceres_pcd$ ls testcfg.yaml
		link to a case yaml
TBD:
	add code to remove floor points.
	test bad matching case 29.

—------- 2/21/22 map_builder.cc—------------------
pose_graph_ :   PoseGraph3D
Trajectory_builders_:  CreateGlobalTrajectoryBuilder3D LocalTrajectoryBuilder3D
?The submap in local traj builder moved/copied to pose_graph_, which is visualized ?

87.17 sec. rvis show trajectory y-axis change, this is  extrapolator.ExtrapolatePose(now) in ros space node.cc. this is not the result of the cartographer's
extrapolator. 

both carto and carto_ros maintain imu extrapolate.

the cartographer code did not keep the extrapolate in the pose_graph.
the rviz plot come from data from Node::PublishLocalTrajectoryData()

the imu data is not immediately processed in carto code. it is delayed to be processed when a new pointcloud data come in. this is the doing of the CollatedTrajectoryBuilder

the imu data saved up in carto is used to extrapolate the init pose for scan match?

-------2/17/22 add use_relative_pose carto mode-----

debugging: 
	icp_match() scan_point_cloud_prev scan_point_cloud
	icp_main2() if both input clouds point to the same cloud ok?

-------2/11/22 imu handling carto -----
files:
	pose_extrapolator.cc
	pose_extrapolator_interface.cc
	internal/imu_based_pose_extrapolator.cc
classes:
	ImuBasedPoseExtrapolator
	PoseExtrapolator

lua option:
	use_imu_based = false

imu data handling:
	PoseGraph3D::AddImuData
	global_trajectory_builder:AddSensorData
	*******LocalTrajectoryBuilder3D::AddImuData
	PoseExtrapolator::AddImuData

skipping high imu:
some scan_hrt_## might be skipped for saving, be aware of seq number.
checking imu data to determine if the current scan should be discarded. this is based on the observation that when robot stop it pitch up down hard, and cause dramatic scan data difference, such as floor points change. so check x/y axis
acceleration.

bad matching:
	scan_hrt_29.pcd  
	submap_test_h_28.pcd
	init: 1.09538 -0.220299 0.401103
	result: 1.03242, -0.811939, 0.39166
		q: [0.962838, -0.0313053, 0.0293501, 0.266647] 
	y-axis should be 0.

@/home/student/Documents/cartographer/test/scan_hrt_29.pcd, the y-axis estimate is bad. this cause large error in y-velocity, and large y-axis position error in
the next 1 sec (where no scan data come in to correct the pose info).
the _29 y-axis seems untractable due to the nature of wall and sensor view field. The only way to avoid 29 is to somehow low weight wall points.
also the explorator_ limiting the velecity change?

-------2/8/22 icp_match lua add new options for carto -----
4 steps to use a new option in lua file:

(1) local_trajectory_builder_options_3d.cc:70
	options.set_voxeledgeratio(parameter_dictionary->GetDouble("voxeledgeratio"));
(2) mapping/proto/local_trajectory_builder_options_3d.proto
	float voxeledgeratio= 23;
	23 is the field id in proto, not actual value
(3 ) local_trajectory_builder_3d.cc
	options_.voxeledgeratio()
(4) trajectory_builder_3d.lua
	voxeledgeratio = 0.5,

new options:
scanmatch_mode: 1 original ceres sm, 3 icp 4 ndt, mode 2 now handle by 
    use_edge_filter option
voxeledgeratio: 
	0-1.0 if p's neighbors number < voxeledgeratio*max_neighbornumb then keep this point.
voxeledgesize: 
	neighbor search radius
pcl_viewerflag: 
	1 display scanmatching cloud and map, 0 no display
use_edge_filter: 
	wether or not use edge filter for scan match cloud, for now only valid for icp/ndt mode scan match
use_relative_pose:
	match scan with previous scan and calculate the pose from relative pose
	this should be impl for all scanmatch_mode
savepcdflag:
	1 to save pcd files, 0 no save. if 1, carto slow down to crawl for b3 dataset.

main interface: 
	must return the pose_estimate.
	ScanMatch()
	ScanMatch_icp()
	
icp_match()
	add icp_example.cc, icp or ndt mode.
	icp_main2() works, result not very good. ndt better than icp
	ndt fail to estimate around scan_hrt_20, @ 82.9 sec
	coincide with odom y-axis angle much higher, floor points
	throw off ndt. 
	
CMakeLists.txt:
	CXX 14, yaml, -Wno-sign-compare
-------2/5/22 VoxelFilterEdge to filter the input scan----
eres_scan_matcher_3d_test.cc


----- 1/29/22 add code for scanmatch_log.txt ---
time, scanfn, init x, y, z, qw, qx, qy, qz, estimated x,y,z, qw, qx, qy, qz
	the init x,y,z is not necessary the previous scan match pose,
	an expolation also contribute.	
traj_0.0.txt:
	each traj pose is almost the scan match estimated pose.
	so a new pose after each lidar data (or scan match call)

------ 1/28/2022 Eigen Quaternion normalize ------------

Eigen Quaternion if not normalized the quat0.toRotationMatrix() give a bad matrix
be careful. see test.cc,
carto code seems normalize the quat.

------ 1/26/2022 b3 scan and submap dump/examination ------------

test/filtered_range_data_in_tracking.pcd-- 8000+ pts --- filtered raw scan (assembled), resolution < 10 cm
test/scan_hrt%.pcd			-- 254 pts   --- filtered of (1)
test/unsync_rangedata.pcd 		-- 384 pts   --- raw scan (one of 160 pieces), no save now
savepcdflag=0;				-- now lua option. for b3, saving pcd file slow it down a lot after 13 secs.
					disable it at normal run. at debugging, pause and go rosbag
					is ok and did not change the outcome.
test:
data gen steps:
	~/Documents/cartographer$ 
	1/2/22 note to use carto 2.0.0
	roslaunch launch/demo_backpack_3d_nobag.launch cartover:=2.0.0
	rosbag play b3-2016-02-02-13-32-01.bag --clock

observation:
	raw scan much large than the one used in scan match, so significant filtering here

	map data quite dense, so map data is build with raw scan, but scan match use filtered
	scan against map.

	carto very slow after about 20 secs. stock version does not have this problem.
	scan_hrt_33.pcd y value =-0.10, and going bad, init_ceres y=-0.03
	scan_hrt_34.pcd y value =-0.25, and going bad, init_ceres y=-0.26
		this indicate bad imu extrapolation during the period?

TBD: further compare with turtlebot data
	modify python filter script to check the result of b3 bag after scan filtering. 
	be aware that b3 bag scan data come in pieces and require assembly in carto node. so
	if simply time interval filtering might cause lose of some scan data.

------ 1/26/2022 pcl_slideshow.py show pcd in sequence ------------
pyenv shell 2.7.17
~/Documents/cartographer$ python pcl_slideshow.py demo2
usage: python pcl_slideshow.py demo1|demo2|demo3 [fnprefix]
# run under ~/Docments/cartographer, pyenv shell 2.7.
# demo1: show all scan*pcd, demo2 show all submap*.pcd
# demo3: show all pcd file with exact name as prefix_###.pcd

observation:
	unfiltered floor points occurs when vehicle stop, probably dip its head
	so see floor more. mapdata_21.pcd, at 82.9 sec
	simple floor point filter is done at baglistener_filterpc2.py

------ 1/25/2022 ceres scan match continue testing ------------
/media/student/data6/catkin_ws/src/cartographer/build
	component tester code
/media/student/data6/cartographer
	main folder to run carto suite.
test/
	submap_test_h_xyzi_hiprob%.pcd
	submap_test_h__xyzi_lowprob%.pcd
	submap_test_h__xyzi_%.pcd
	submap_test_h%.pcd   --- this is the map before scan_hrt%.pcd insertion
	scan_hrt%.pcd
	scan_hrt.csv	: gt odom data (python code)
	scanmatch_log.txt: carto node log file
	scanmatch_log_odom.txt: combined log file (scanmatchlog_combine.py)

test_ceres_pcd/1-25-22/
	copy of test/*
test_ceres_pcd/1-25-22/bad/
	submap/scan snapshot for bad match
test_ceres_pcd/3-8-22-turtlebot3_imuodompt2_4
test_ceres_pcd/3-15-22-mavrosshort

observation:
	last few frame seems have good result, double check with ceres...tester
	compare it with gt value.
	the est is pretty good. rotaton good. translation  odom adjust offset
	bad match:
		87.5 sec - 90.7 sec: scan_hrt_35.pcd scan_hrt_41.pcd
	see floor points in scan_hrt_20.pcd ... scan_hrt_30.pcd
	these floor points get into submap and cause matching problem.
	
Video:
	code modify, build, data run, ceres test:
	turtlebot3_gazebo_imupt2_carto_3.mp4
Ref:
	1/4/2022 note

------- 1/7/22 test.cc compile_testi2.sh --------------
test.cc:
	add some Eigen test code
------- 1/7-14/22 carto scanmatch debug ceres --------------
cartographer/test_ceres_pcd
	scan_test_h.pcd --- saved at ScanMatch call
	test/submap_test_h.pcd --- saved at ScanMatch call
	readme.txt
	sweep3.ods  --- costfunc plot of sweeping the init pose 
	gnuplot splot.p
	sweep4.csv
	sweep4.png
	runplot.sh
	
ceres_scan_matcher_3d_test.cc
	new test code to scanmatch the above two pcd clouds.
	show two clouds together before and after

    ref:
	/media/student/data6/catkin_ws/src/cartographer/readme.txt
	

two snap shot pcd file: 
pcl_viewer  -fc 0,0,255 scan_test_h.pcd  -fc 0,255,0 submap_test_h.pcd
cd /media/student/data6/catkin_ws/src/cartographer/build
	./cartographer.mapping.internal.3d.scan_matching.ceres_scan_matcher_3d_test

ceres_scan_matcher_3d_test usage
status: 
	ceres_scan_matcher_3d_test scanmatch result not as expected.
		could be resolution and scanmatch setup.
		after get a reasonable result, try to fine tune
		ex. submap with miss points removed.
	icp_example match result also show the impact of the miss points.
	
	hgrid_res: 0.2 in testopt2.txt
	main code add lua option: hgrid_pcd_probthresh = 0.5
		will exclude most miss point to be dumped to pcd file
		for test.
	hgrid resolution affect ceres scan match. if too small, not work at all.
	increase 0.5 allow ceres to find a close one. but low resolution
	result in less accurate result.

	add low resolution submap and scan in the residue block: hopefully the 
		low resol will search to a rough match, then high resol part
		further increase the match result.
	

	scanmatchtest now has three mode of scanmatch: (1) high reso, (2) low resol, (3) both resolutions. (4): low resol first, then high reso, not together. option twostep: 1|0. the grid size and initial pose must be similar. 
	the result of (3) and (4) same? 

	add code to perform sweep gride size, 0.5 vs 0.15
		cost func plot see sweep3.ods. at low hgrid res, it is smooth
		at fine resolution, the cost fun have many local mins.
		this explained why ceres opt fail and inconsisitant
		at high res grid size
		specific ex where low+high result in a great result:
		initpose: 0.65 0.0278726 -0.0228209 
		-0.096398 0.111561 -0.0720986 (low+high)
		-0.174376 -0.0790631 -0.198817 (low resolution)
		0.403925 0.382077 -1.66027 (high resolution), not even close

	sweep with both x and y and plot: sweep4.ods
	test with actual high/low resolution data
	gnuplot splot surface
	
------- 1/4/22 carto scan hybrid_grid submap data generation debug ----------
local_trajectory_builder_3d.cc:
	hybridgrid_2_pcd()
	scan_2_pcd()
	submap_2_pcd()

dump the scan and submap to pcd files for examination:
	- before ScanMatch, the input scan are voxel filtered. so it is
	sparse than the raw input cloud.
	- the submap dumped indicate bad alignment, turtlebot data
	  show multiple walls while there should be a thin wall
	turtlebot3 start move the 2 sec. yet submap already show 3 unaligned
	slice at 3 second. check insert point code

debug code/files:
  ~/Documents/cartographer/
	rostopics:
		/scan_matched_points2   data(3) filtered
		/camera/depth/points2	data(0) orig resolu
	sync_rangedata.pcd: data(0) 76800 pts, original resolution
	test/submap_test_l%.pcd
	test/submap_test_h%.pcd   --- this is the map before scan_hrt%.pcd insertion

		submap hybrid_grid dump, high reso, update each scan 
		contain both hit and miss points. to exclude miss points
		hgrid_pcd_probthresh = 0.5, otherwise =0.1
		see hybridgrid_2_pcd(), 900 pts, data(3)x3
		new option hgrid_pcd_probthresh
			in conf...files2/trajectory_builder_3d.lua
	test_ceres_pcd/1-25-22 hgrid_pcd_probthresh = 0.5
		upto submap_test_h_11.pcd shows two layers
	test_ceres_pcd/2-1-22 hgrid_pcd_probthresh = 0.1
		submap_test_h_1.pcd already has 3 layers. two layers are
		the miss points layer.

	scan_test_l.pcd:	why is this file slightly large than scan_test_h.pcd?
				1/24/2022, the # of points dep on min_num_points, for some
				reason its 200 for low_res and 150 in high_res in the lua config
				max_length of voxel make sense though.
	scan_test_h.pcd:	current scan dump, called inside ScanMatch()
				starting from 2nd scan data, data(1)
	test/scan_hrt_%.pcd:	current scan filtered in AddAccumulatedRangeData(), data(1), 163 pts
				1-7 odom 0s, 8.pcd x-axis 0.08 meter
	test/scan_hrt.csv:	odom info at the time of scan_hrt_%.pcd, saved by baglistener_filterpc2.py
	test/filtered_range_data_in_tracking.pcd:
				data(3), 348 pts
data generation step:
	11/29/2021 note "test with carto:"
	1/1/2021

code building step:
  (ros pkg)
	src: /media/student/data6/catkin_ws/src/cartographer/cartographer$ 
		vi mapping/internal/3d/local_trajectory_builder_3d.cc
	build: /media/student/data6/catkin_ws
		catkin_make_isolated --install --use-ninja
  (carto test)
	build: /media/student/data6/catkin_ws/src/cartographer/build
		ninja
		./cartographer.mapping.internal.3d.scan_matching.ceres_scan_matcher_3d_test
	
cloud data processsing/flow:
	data (3)(2)(1) all very similar. all sparse
	data(0) unsynchronized_data.ranges, 
		sync_rangedata.pcd: data(0) 76800 pts, original resolution
		76800 pts. 
		passed from ros 
		PointCloud2 converted to carto::TimedPointCloudData. 
	data(1) high_resolution_point_cloud_in_tracking, 163 pts
	data(1.b) low_resolution_point_cloud_in_tracking, 211 pts
	data(2) filtered_range_data_in_local,
	data(3) filtered_range_data_in_tracking, filtered, 348 pts
	processing: 
		data(0) is first filtered from orig resolu to data(3), much sparse.
		data(0) is /camera/depth/points2
		data(3) is /scan_matched_points2
		func (b, f): data(3) ---adaptive filter voxel --> data(1), (1.b)
		func (b): data(3) ---frame trans  --> data(2) 
		func (e): data(0) --- voxel filter ---> data(3)
	LocalTrajectoryBuilder3D::
	(a) --call--> (e) (b)
	(b) --call --> (c) (d) (f)

	(a) AddRangeData(data(0))
	(b) AddAccumulatedRangeData(data(3))
	(c) ScanMatch(data(1))
	(d) InsertIntoSubmap(data(1,2,3))
		(d.1) active_submaps.InsertData()
		when insert scan data to submap, it is not really "adding" the data directly.
		it actually update the value at the correspoinding grid points for high_res and i
		low_res hybrid_grid. so the same range data is used for both high and low res hgrid,
		not the high_res and low_res filtered data.
		(d.2) InsertionResult()
		it also update the graph, which point to the scan data.
 
	(e) VoxelFilter(), voxel_filter_size, 0.15, voxel is like a cubic cell
	(f) AdaptiveVoxelFilter, use max_length , min_num_points to filter
		high_resolution_adaptive_voxel_filter_options
		low_resolution_adaptive_voxel_filter_options

options:
	map_builder.lua
	pose_graph.lua
	trajectory_builder.lua
	trajectory_builder_3d.lua

submap creation/update:
	submap point probability change as new data is inserted. hit pts
	start with 0.55, miss pts start with 0.49, if the same hit
	pts are observed again in new scan, the prob goes up. similary,
	if the same miss pts are observed, the prob goes down.
	after 10 or so scan, hit pts typically > 0.7, miss pts prob <0.3
	hit pts mean obstacle, miss pts mean no obstacle.

   mapping/3d/range_data_inserter_3d.cc:
	InsertMissesIntoGrid()
	due to the insertion of the miss points, the submap has close to 3x
	points than scan. the miss points prob 0.49, the hit points prob 0.55
	when responding to submap inquery, ExtractVoxelData() only return points
	whose prob>0.501f (see 3d/submap_3d.cc)

------- 1/3/22 carto scanmatch debug --------------
debugging info:
	mapping/internal/3d/scan_matching/
		ceres_scan_matcher_3d.cc
		local_trajectory_builder_3d.cc
use 1/1/22 filter to 'slow down' freq of points to observe match result.
print fullreport and estimated pose difference.
	turtlebot3 bag pub pc2: 60 hz.
	carto still work with skipcnt=60 or 120

TBD:
	dump submap to pcd file
	local_trajectory_builder_3d.cc
	cartographer/io/pcd_writing_points_processor.cc

ScanMatch initial pose: from imu projection, converted to body frame
  submap contain pose data, hybrid_grid don't have pose data.
  so when match pc to hybrid_grid, the initial pose must be in body frame.
  in turtlebot data, the first submap pose overlap the odom frame 	
  submap version # update (+1) after each pc insertion. 

------- 1/2/22 carto pkg build tip --------------
tip: PCL package must be added to CMakeLists.txt for 
	if want to use PCL's full library (such as PCD)
	cartographer
	cartographer_ros
	cartographer_rviz

------- 1/2/22 carto submap debug --------------
see 11/12/2021 cartographer bag study, submap

----------1/1/22 turtlebot3 pt2 frame filter -------------
python code to control which pc2 frame go through.
skipcnt: param, how many pc2 to skip
	turtlebot3 bag file 60hz for pc2 

test step:
	roslaunch launch/turtlebot3_3d.launch cartover:=2.0.0
	pyenv shell 2.7.17; rosparam set skipcnt 20; python baglistener_filterpc2.py
	rosbag play turtlebot3_imuodompt2_3.bag --topics /imu /camera/depth/points /odom /camera/depth/points:=/tmppt2 /imu:=/mavros/imu/data /odom:=/odom_gt --clock

filter:
	baglistener_filterpc2.py  --- repub pt2 topic after filter turtlebot bag files
	baglistener_filterpc2_b3.py  --- repub pt2 topic after filter for b3 bagfiles

------- 1/2/22 carto stock version 1.0.0 vs build 2.0.0 --------------
build from source version 2.0.0
	lua file different. if launch file use 1.0.0 lua file, error
	about certain parameters if use 2.0.0 carto node.

	rosversion cartographer

to use the nonstock carto version:
	export ROS_PACKAGE_PATH=
	source /media/student/data6/catkin_ws/devel_isolated/setup.bash
	then launch as:
		roslaunch launch/demo_backpack_3d_nobag.launch cartover:=2.0.0

fix: launch file check carto version.
------- 12/29/21 ceres-solve --------------
homepc:Documents/ceres-solver
Build:
	mkdir ceres-bin; cd ceres-bin; cmake ..; make
Test:
	cd ceres-bin/bin; ./hellowworld

test Jet class

----- 12/29/21 pcl python code -------------------
verifypose.py
	curr fold must contain yaml and pcd files
	convert pcd file based on pose info in yaml
visualization.py
	show pcd file like pcl_viewer
pcl_slideshow.py
	slide show of pcd files in the folder

------- 12/21/21 verify pose info / apply to pcd files --------------
script:
	verifypose.py: calculate relative transformation between two pair
		of (pc_0, posei_0) and (pc_1, posei_1)
	filter_floor.py: remove floor points from pcd file

Pcl_viewer to view two clouds
Pcl_viewer  -fc 0,0,255 mapdata_6.pcd  -fc 0,255,0 test.pcd

--------------12/20/21 python-pcl build-------------------------
homepc, lenova1 (2/28/22)
python binding of pcl library. not upto date maintained.
prebuild pkg is for pcl1.7, system has pcl1.8
	pkg-config --list-all|grep pcl
build from source have link problem
workaround by change setup.py
build/install for 2 pyenv: 2.7.17 and miniconda

Build step:
	pyenv shell miniconda3-latest
		or 2.7.17
		pyenv install 2.7.17
	git clone the repo.
	cd python-pcl
	modifiy setup.py
		cp ~/Documents/miscfiles/python-pcl/setup.py .
			https://github.com/juwangvsu/miscfiles.git
	python setup.py install
		this will compile and install @ current python env.
		2.7 env might error when install filelock pkg. seems safe
		to ignore, rerun pyenv shell 2.7.17
ref:
	"readme pointcloud ndt icp slam cartographer code "
	2.7 pyenv also add some rospy pkgs: pyyaml rospkg rospy
		rosdep rosinstall_generator wstool rosinstall wheel 
		pycryptodome pycryptodomex gnupg cython	
	
------- 12/20/21 generate point cloud odom for deepvo --------------
lenova2:
from turtlebot3_imuodompt2_3.bag bag file
results:
	.pcd; yaml; .ply
	raw ply or pcd  data is w.r.t the camera sensor frame. y-axis top to bottom, z-axis point into the screen. x-axis left to right
	at mapdata_21.pcd, y-axis dip due to robot stop.
	pcl_viewer mapdata_22.pcd mapdata_21.pcd -use_point_picking


  take pc2 and odom data, synced
        (1) cd test/turtlebot3_imuodompt2_3; rosrun hdl_graph_slam savecloud
        (2) ~/Documents/cartographer/; ./savecloud.sh
        (3) ~/Documents/cartographer/; rosbag play turtlebot3_imuodompt2_3.bag --topics /odom /camera/depth/points /camera/depth/points:=/filtered_points
                wait for complete, 
        (4) test/turtlebot3_imuodompt2_3$ ../../pcd2pcl_clean.sh
                this convert pcd to ply files

ref: 12/11/21 note (2)

------12/14/21 convert pcd file to ply  ---------
	ply to be open by meshlab, to manuelly remove
	floor points to test icp or ceres
	cartographer/test/mavros_realsense_pcd$ ../../pcd2pcl_clean.sh
	meshlab mapdata_45_.ply
	click "vertex select" button, now use mouse to highlight the area
		of vertex, click "delete selected vertex"
		save to ply (select ascii format)
	
------12/11/21 hacking ceres_scan_matcher_3d_test.cc ---------a
homepc
/media/student/data6/catkin_ws/src/cartographer`
to test pcd files
        - add pcl library to the CMakeLists.txt
        - it seems work. pcl code works fine inside the test code
to rebuild:
        cd build; cmake ..; ninja
to run test:
        cd build
        ./cartographer.mapping.internal.3d.scan_matching.ceres_scan_matcher_3d_test

	ceres weakness:
		very few points (e.g., 2 or 3 points no good)
		undistinguishable points
TBD:
	ceres in carto vs  ceres in test code.

ref: my repo cartographer-1's readme

------- 12/11/21 point cloud examination --------------
scan match pc2 obs:
	-using icp_example for scan match
	-turtlebot pc2 result good if two pc2 relative close
	-mavros_realsense_short pc2 result bad
TBD:
	- remove floor points for mavros_realsense_short pc2	
	- further check carto scan matching part, if possible 
	  seperate that code for testing on pcd files. 

(1) pcd files from pc2 topic:
       rosparam set /savemap_node/savecloud true
        # run savecloud.sh to save 100 pcd files.
        # or do this multiple times while pause/run bag for multiple pcd files
      rosparam set /savemap_node/savecloudonly true
        this param is used in hdl_graph_slam/savecloud
        to use only point cloud. else it will save odom too.
      cd test; rosrun hdl_graph_slam savecloud
      rosbag play mavrosshort_transrecord.bag --topics /mavros/imu/data /camera/depth/points2  /camera/depth/points2:=/filtered_points
      or rosbag play turtlebot3_imuodompt2_3.bag /camera/depth/points:=/filtered_points --clock
           saved pcd: mapdata_3,4….pcd
           view pcd: pcl_view mapdata_3.pcd

(2) get ~100 pcd from bag file: (0.3 sec interval)
	(a) cd cartographer/test/turtlebot3_pcd
        (b) rosparam set /savemap_node/savecloudonly true
		for mavrosshort_transrecord.bag
	(c) rosrun hdl_graph_slam savecloud 
	(d) cartographer$ ./savecloud.sh 
	(e) rosbag play turtlebot3_imuodompt2_3.bag /camera/depth/points:=/filtered_points --clock

(3) Run scan match:
	cd /media/student/data6/cartographer/test/
	~/Documents/hdl_graph_slam/ndt/build/icp_example -pst=1 -if=mapdata_3.pcd -tf=mapdata_4.pcd -yawg=0.1 -xg=0 -rsl=0.02 -md=gicp -ms=-1
		-tf: target frame, -if: reference frame, i
		result translation is to move tf to if
	~/Documents/hdl_graph_slam/ndt/build/icp_example -pst=1 -rsl=0.02 -md=ndt -ms=-2
		ms -2: read both pcd file names and init pose from testcfg.yaml

	turtlebot3_pcd$ ~/Document/hdl_graph_slam/ndt/build/icp_example -pst=1 -if=mapdata_32.pcd -tf=mapdata_46.pcd -yawg=0.1 -xg=0 -rsl=0.02 -md=gicp -ms=-1

mappose_46.yaml
	quat: [ -0.000706, 0.0011278, 0.5312124, 0.8472376 ]
	xyz: [-1.4950, 1.7832, -0.0010]
	euler: [ x: -0.1371952, y: 0.066518, z: 64.1749599 ]
mappose_32.yaml
	quat: [-0.0006 , 0.00116, 0.488896, 0.87234]
	xyz: [-1.8752, 0.99411, -0.00101]
	euler: [ x: -0.1249652, y: 0.082343, z: 58.5362951 ]a
icp_example result:
	R=[
   	0.995116 0.000250801    -0.09871  -0.0950297
	-0.00025028           1 1.78665e-05 -0.00231355
  	0.0987101 6.92345e-06    0.995116    0.875233
          0           0           0           1
	]
	quat [-2.73911e-06 -0.0494154 -0.000125424 0.998778]
	euler: [ x: -0.0010287, y: -5.6648966, z: -0.014441 ]
	xyz trans: [-0.0950297 -0.00231355 0.875233]
(3.b) 
	mavros_short pc2 #3 and #14 also pretty good.
	mavros_realsense_pcd$ ~/Documents/hdl_graph_slam/ndt/build/icp_example -pst=1 -if=mapdata_3.pcd -tf=mapdata_14.pcd -yawg=0.1 -xg=0 -rsl=0.02 -md=gicp -ms=-1
	
(4) turtlebot3 point cloud ros msg ex:
 from sensor_msgs.msg import Imu, PointCloud2
# header:
#  frame_id: "camera_depth_optical_frame"
# height: 480
# width: 640
# fields" x,y,z, rgb
# point_step: 32
# row_step: 20480
# data: [235 ...]

obs:
	the icp result is good for mapdata_32.pcd and mapdata_46.pcd
	note the point cloud is on camera frame, whose z-axis is 
	the odom frame x-axis, and its y-axis is odom's -z-axis .so the 
	rotation calculated by icp_example is on y-axis
ref:
	hdl_graph_slam/readme, 3/9/21 note
	"airsim gmapping hdl slam hector cartographer ndt icp robot_localization open3d", 12/11/21

--------12/9/21  mavrosshort record 2 -----------
  imu data z unchange in baglistener_mavrosshort2.py 
  filter out msg with time glitch
  bag record:	
	- roscore; rosparam set /use_sim_time true
	 -rosbag record -O mavrosshort_transrecord_2.bag /mavros/imu/data /camera/depth/points2	
        -roslaunch depth_image_proc/depth_image_tmppt2.launch
	 - python baglistener_mavrosshort2.py
        -rosbag play mavros_realsense_short.bag /mavros/imu/data:=/tmpimu --clock
  status: carto no longer crash, but result no good.

--------12/9/21  mavrosshort carto debug -----------
mavrosshort bag file
p3at_3d_90deg.urdf camera link fixed , imu x-axis = base x-axis
	but result still very bad.
test10.csv
	check imu data direction.
	python imu2cvs.py test10.csv
	rosbag play mavrosshort_transrecord.bag

	record imu data to do some rough traj estimate by eye.
	
imu2csv_mavros.py
test11.csv
	python imu2csv_mavros.py test11.ods
	rosbag play mavros_realsense_short.bag
	progressing: convert /camera/imu and /mavros/imu/data to cvs
		from mavros_realsense_short.bag and compare xyz-acc

		observed /mavros/imu/data and /camera/imu have very different
			time stamp value, this might also be the reason
			for carto fail and crash. 
			nope. mavrosshort_transrecode.bag's pt2 have the 
			same time stamp as clock and imu data. 
			crash reason:
				/camera/depth/points2 time stamp glitch
				 nsecs going backward.
				this glitch @ mavros_realsense_short.bag
				/camera/depth/image_rect_raw
				    secs: 1636502986
    				    nsecs: 558187962
    				    secs: 1636502986
    				    nsecs: 557096004

see 11/26/21 note for view and carto

----------1/2/22 turtlebot3 pt2 frame filter -------------
python code to control which pc2 frame go through.
skipcnt: param, how many pc2 to skip
	turtlebot3 bag file 60hz for pc2 
roslaunch launch/turtlebot3_3d.launch cartover:=2.0.0
osparam set skipcnt 20; python baglistener_filterpc2.py
rosbag play turtlebot3_imuodompt2_3.bag --topics /imu /camera/depth/points /odom /camera/depth/points:=/tmppt2 /imu:=/mavros/imu/data /odom:=/odom_gt --clock

----------12/9/21 turtlebot3 carto imu_link update ---
turtlebot3_imuodompt2_3.bag
turtlebot3_3d.urdf:
	imu_link_joint no rot 

test with carto:
	roslaunch launch/turtlebot3_3d.launch
	roslaunch launch/turtlebot3_3d.launch cartover:=2.0.0
		if run non stock version of carto
	rosbag play turtlebot3_imuodompt2_3.bag --topics /imu /camera/depth/points /odom /camera/depth/points:=/camera/depth/points2 /imu:=/mavros/imu/data /odom:=/odom_gt --clock

result:
	carto result up to 7.5 sec good 
	carto scan match no good from 11. when do turtle rotate 45 deg
	carto result ok during the second seg of forward movement
Video:
	turtlebot3_gazebo_imupt2_carto_2.mp4

ref:
	11/29/21 note 

---------- turtlebot3_imuodompt2_3.bag imu odom examination --------
movement pattern observation:
	xy_odom plot: start time 75.5 sec,
	move forward (x-axis), rot 45 deg, move forward
	turtle only capable of x_body move, no y_axis movement
	start move at 77.4 sec ( x-acc +), 
	stop at 82.8800 sec (x-acc -), robot body dip, point cloud dip y-axis
		mapdata_21.pcd
	rot, move again at 92.2 sec, stop at 97.7 sec, notice 
	(x-acc_body >x-acc_odom) due to orientation change
	meanwhile y-acc_body < y-acc_odom 

test9.ods:
	shows that odom and imu are consistent in x-y-z values. imu x-acc is align with odom x-axis value. the stock urdf in gazebo run actually has base_link and imu_link almost overlapping. 
in current carto and view setup for this bag file, the urdf has imu_link 180 deg x-axis rot from base_link, thus the inconsistent between imu data and odom.

-------------12/7/21 bag file conversion imu data cross ref ----
b3-2016-02-02-13-32-01.bag -
	--filter to keep two topics 
	  --> b3_rerecord.bag i
		--> change frameid 
		  --> b3_transrecord.bag
			carto: p3at_3d_b3bag.launch
			base_link->imu_link [-180.000, -0.000, 0.000]
   imu z-acc = -9.8
   baglistener.py -> b3_transrecord.bag

turtlebot3_imuodompt2.bag
	- recorded from gazebo
	     carto: turtlebot3_3d.launch
	     base_link->imu_link [-180.000, -0.000, 0.000]
   imu z-acc = 9.8

mavros_realsense_short.bag
   imu z-acc =9.8
	--> change frame id, neg z-acc value
	mavrosshort_transrecord.bag
   		imu z-acc = -9.8
   baglistener_mavrosshort.py

---------12/6/21 urdf, bagfile, launch file cross ref ---
backpack_2d.urdf
backpack_3d.urdf
p3at_3d_90deg.urdf
        bag: mavrosshort_ bag files
        lidar x-axis -90 deg rotate followed by z-axis -90 deg
        p3at_3d_90deg.launch
        p3at_bringup_tf.launch
p3at_3d.urdf
        bag: b3_...bag files.
        p3at_3d_b3bag.launch
        p3at_3d.launch
turtlebot3_3d.urdf
        bag: turtlebot3_imuodompt2_3.bag
             turtlebot3_imuodompt2_4.bag
        simplified with 3d lidar
        turtlebot3_3d.launch (carto)
	turtlebot3_slam/launch/turtlebot3_bringup_tf_simpleurdf.launch (view)
turtlebot3_simple.urdf
        simplified with 2d lidar
        turtlebot3_2d.launch

bagfile info:
        turtlebot3_imuodompt2_3.bag	horizontal_fov 1.3439, 320x240 30hz
        turtlebot3_imuodompt2_4.bag	horizontal_fov 2.6878, 320x120 20hz

----12/5/21 mavrosshort_.bag carto test ---
mavrosshort_transrecord.bag
see 11/26/21 note

	imu_link y-axis = base_link x-axis? this is wrong. from rviz viewing the data.

----12/4/21  turtlebot3_imuodompt2_3.bag carto test fov---

changed turtlebot3_waffle.gazebo.xacro
	<sensor type="depth" name="realsense_R200">
		<width>320</width>
              <height>240</height>
this seems help a bit. still no good:
	- point still too dense?
	- floor point causing problem?

camera field of view:
<horizontal_fov>1.3439</horizontal_fov>
	the horizontal_fov decide the horizental field of view, this further decide the vertical fov by scale by w/h resolution ratio. fixing hori fov, change the w/h ratio affect how much vertical content will be see, and the resolution of the data points.


see 11/29/21 note prev test run

----12/4/21 turtlebot urdf gazebo small bugs ---

fixed urdf bug: camera_link and camera_rgb_frame in gazebo small movement due to joint limit

bug remained:
	in gazebo, when stoping robot, it will have a noticable rotation
	like type skidding. possible causes: cast joints friction...

----12/2/21 python code to calculate odom from raw imu data turtlebot bag ---
 
assume orientation know from the odom topic.
	python odomimu2csv.py test3.csv
		convert imu and odom topic to csv
	test3.csv: 
		converted from topics from turtlebot3_3_gazebo.bag
	test9.csv: 
		converted from topics from turtlebot3_imuodompt2_3.bag
	odomimutf_example.py
		code sniplet tf quaternion

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
More data about imu's ax, yx,..
	roslaunch turtlebot3_slam/launch/turtlebot3_house_bringup.launch
	python odomimu2csv.py test.csv
test4.ods: rotation clockwise
test5.ods: rotation counter-clockwise
test6.ods: forward movement, x-acc positive during ramping up, then negative when winding down.
test7.ods: backward movement, x-acc negative during ramping up, then positive when winding down.
Y-acc also exist, but much smaller than x-acc
test8.ods: forward movement, orientation 45 deg z-axis. ax ay similar to test6, as expected. ax_o ay_o vx_o vy_o (odom frame) seems reasonable. the value drift off quite large after robot stop.

	
----11/30/21 verify carto result with 'ground truth'------------------
data:
	turtlebot3_imuodompt2.bag	36 sec
	turtlebot3_imuodompt2_2.bag	33 sec, forward, left 90deg, forward.
	turtlebot3_imuodompt2_3.bag
		 depth is z-axis of camera_depth_optical_frame

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

play view bag:
	*roslaunch turtlebot3_slam/launch/turtlebot3_bringup_tf_simpleurdf.launch
		simpleurdf is the same urdf as turtlebot3_3d.launch
	roslaunch turtlebot3_slam/launch/turtlebot3_bringup_tf.launch
		use stock turtlebot3 urdf wt gazebo tag
	rosbag play turtlebot3_imuodompt2.bag --clock
	rosbag play turtlebot3_imuodompt2_3.bag --clock

test with carto:
	roslaunch launch/turtlebot3_3d.launch
	roslaunch launch/turtlebot3_3d.launch cartover:=2.0.0
		if run non stock version of carto
		to use nostock version, see 1/2/22 
	(1/2/22 optional) rosparam set skipcnt 20; python baglistener_filterpc2.py

	rosbag play turtlebot3_imuodompt2_3.bag --topics /imu /camera/depth/points /camera/depth/points:=/camera/depth/points2 /imu:=/mavros/imu/data --clock

	carto node work, result no good. pt2 seems at z-axis, rotate needed? 
	fixed:	create corresponding urdf for simplified turtlebot
		turtlebot3_3d.urdf camera_depth_optical_frame_joint
			  rpy="1.571 0. -1.571"
	pt2 dispaly slugish, point cloud too dense?
		play-pause bag file make cpu easy.
	carto trajectory good up to 16 sec. at 17 sec the pt2 change too much
		cause scan matching fail and result in bad traj.
	turtlebot3_imuodompt2_2.bag result also have trouble. even the robot
		movement and data suppose to be easy. the matching alg might
		have trouble with dense pt2

videos:
	turtlebot3_gazebo_imupt2.mp4 -------- turtlebot3_house_bringup.launch
	turtlebot3_gazebo_imupt2_recordbag.mp4 -------- record turtlebot3_imuodompt2.bag
	turtlebot3_gazebo_imupt2_carto.mp4 -------- first carto test with turtlebot3_imuodompt2_2.bag 
		
 

see 9/4/2021 note

----------11/26/2021 b3 bag imu taint experiment  -----
setting TRAJECTORY_BUILDER_2D = {
  use_imu_data = false,
seems not affecting the carto result

taint imu effect limited, neg y,z of the imu data seem flip the resultant
trajectory by rotate x-axis 180 deg
adding offset to z or neg z only no effect.

----------11/26/2021 mavros short bag debugging -----
mavrosshort_transrecord.bag:
	/camera/depth/points2	sensor_msgs/PointCloud2
	/mavros/imu/data	sensor_msgs/Imu
transcode mavros_realsense_short.bag to have the same topics and
	frame_id as b3_transrecord.bag. mavros bag file contain /mavros/imu/data
 	but frame_id is base_link; it also have depthimage: also imu z gravity
	must be negative.
	also convert depth/image_rect_raw to depth/points

  bag record:	
	- roscore; rosparam set /use_sim_time true
	 -rosbag record -O mavrosshort_transrecord.bag /mavros/imu/data /camera/depth/points2	
        -roslaunch depth_image_proc/depth_image.launch
	 - python baglistener_mavrosshort.py
        -rosbag play mavros_realsense_short.bag /mavros/imu/data:=/tmpimu --clock

  view bag file:
	roslaunch launch/p3at_bringup_tf.launch 
		p3at_3d_90deg.urdf
	rosbag play mavrosshort_transrecord.bag --topics /mavros/imu/data /camera/depth/points2  /mavros/imu/data:=/imu /camera/depth/points2:=/camera/depth/points --clock 
	rosbag play b3_transrecord.bag --topics /mavros/imu/data /camera/depth/points2  /mavros/imu/data:=/imu /camera/depth/points2:=/camera/depth/points --clock 

  test:
	- camera_depth frame rotated so the pt looks normal at base x-axis
		p3at_3d_90deg.urdf
	- roslaunch launch/p3at_3d_90deg.launch
	- rosbag play mavrosshort_transrecord.bag --clock
	- rosbag play mavrosshort_transrecord_2.bag --clock
	status:
		fixed:	carto node seems received the imu and pt2 data
		but fail to calculate, another issue with clock
		within data? 

		fixed: the b3_xxx bag pt2 msg only contain a small
			section, so lua param num_accumulated_range_data =160
			change to 1.
		the carto node is calculating and
			publish map->odom tf, but still crash.
			the map and traj produced are wrong. scan match result bad
			also seem the imu y-axis is inline with robot x-axis.

Video:
	mavrosshort_carto.mp4
			

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
	- rosbag play b3-2016-02-02-13-32-01.bag --clock

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

	view bag file:
		roslaunch launch/p3at_b3bag_bringup_tf.launch
		rosbag play b3_transrecord.bag --topics /mavros/imu/data /camera/depth/points2  /mavros/imu/data:=/imu /camera/depth/points2:=/camera/depth/points --clock 

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
view mavros bag file:
	roslaunch turtlebot3_slam/launch/turtlebot3_bringup_tf.launch
	rosbag play mavros_realsense_short.bag --topics /mavros/imu/data /camera/depth/image_rect_raw /camera/depth/image_rect_raw:=/camera/depth/image_raw /mavros/imu/data:=/imu --clock


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

----------11/12/2021 cartographer bag study, submap-----
3d:
	/horizontal_laser_3d
	/vertical_laser_3d

	pub:
	/scan_matched_points2
	/submap_list
		each trajectory point have one associated submap, 
		labeled: trajid, submap index id, version #: 0,0,320

to obtain a trajectory:
	carto 2.0.0
	rosservice call /trajectory_query "trajectory_id: 0" > traj_0.txt
		the list is the scan match estimated pose
to obtain a submap:
rosservice call /submap_query "trajectory_id: 0
submap_index: 0"  > submap_0.0.txt
	submap_0.0.txt: contain high resolute (0.1 meter) 
		and low resolute (0.45 meter), cell array cells: [31, 139, ...]
		with pose, width, height
		cell array is compressed string. 
		debug cartographer_rviz plugin to dump uncompressed looks good.
	ex:
cell: reso/w/h: 0.45 9 10
alpha:
0 0 0 0 0 12 15 31 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 44 44 44 44 44 44 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 

debug code:
cartographer_ros/submap.cc	--- ros code to get submap data
cartographer/mapping/3d/submap_3d.cc --- hybrid_grid to submap conversion
std::string MapBuilder::SubmapToProto() --- gen response to query
	submap_dbgflag //defined in 3d/submap_3d.cc
	submap_query roscall with submap_id.submap_index>1000 or 2000 to
		turn on/off dbgflag
rosservice call /submap_query "trajectory_id: 0 submap_index: 2001"	
rosservice call /submap_query "trajectory_id: 0 submap_index: 1001"	

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

	source install_isolated/setup.bash
		to run carto node build here
	PCL link issue: 
	
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
