
3-15-22, mavros bag output:

PoseGraph3D::AddImuData: -0.321835 -0.718341 10.0096 time: 637720997679578203
PoseGraph3D::AddImuData: -0.350435 -0.683835 10.0181 time: 637720997679778203
PoseGraph3D::AddImuData: -0.339802 -0.688168 9.98077 time: 637720997679978129
PoseGraph3D::AddImuData: -0.360519 -0.697814 10.0118 time: 637720997680178129
PoseGraph3D::AddImuData: -0.352235 -0.69904 10.0172 time: 637720997680378129
PoseGraph3D::AddImuData: -0.328024 -0.68643 10.0231 time: 637720997680538129
PoseGraph3D::AddImuData: -0.335322 -0.705555 10.0016 time: 637720997680778129
PoseGraph3D::AddImuData: -0.349485 -0.696839 10.019 time: 637720997680978054
PoseGraph3D::AddImuData: -0.328495 -0.650497 10.0236 time: 637720997681178054
PoseGraph3D::AddImuData: -0.320743 -0.696573 10.0014 time: 637720997681338054
PoseGraph3D::AddImuData: -0.330984 -0.698449 10.0591 time: 637720997681578054
PoseGraph3D::AddImuData: -0.330163 -0.686585 10.0262 time: 637720997681778054
PoseGraph3D::AddImuData: -0.33454 -0.685573 10.0086 time: 637720997681937978
******LocalTrajectoryBuilder3D::AddAccumulatedRangeData
PoseExtrapolator::GetImuData imu_data_.empty() 0
GetImuData front time: 637720997679378203 imu_data_.size(): 14
GetImuData back time: 637720997681937978
scan_2_pcd #points: 595 /home/student/Documents/cartographer/test/filtered_range_data_in_tracking_6.pcd

******   high_resolution_adaptive_voxel_filter_options.min_num_points ******
150scanmatch_mode: 4
mode 3/4, use_edge_filter(): 0
*******ScanMatch_icp call
*******ScanMatch_icp first entry


submap_2_pcd: { t: [0, 0, 0], q: [1, 6.93889e-18, -2.32934e-21, -2.1684e-19] }
***** hybridgrid_2_pcd probthresh: 0.5
	submap info: HybridGrid #points/hit#/miss#: 3918 1646 2272 max/min prob: 0.731704 0.450143
	submap # points: 1646 submap_test_h_6.pcd

***** hybridgrid_2_pcd probthresh: 0.5
	submap info: HybridGrid #points/hit#/miss#: 525 299 226 max/min prob: 0.731704 0.450143
	submap # points: 299 submap_test_l_6.pcd
scan_2_pcd #points: 159 /tmp/tst.pcd
icp_match:  scan_point_cloud, 0x5568f6fb7720 icp_match: scan_point_cloud.width 159
icp_match: hgrid_point_cloud, 0x5568f6fc5570 0x5568f6fc5570
icp_match: hgrid_point_cloud/scan_point_cloud w/h: 1646 1 159 1
icp_match: initial_ceres_pose.translation: 0.0166901 0.235983 0.0439406
using ndt verison
init matrix:
  0.997965  0.0353858  0.0530502  0.0166901
-0.0384725   0.997555   0.058339   0.235983
-0.0508562 -0.0602612   0.996886  0.0439406
         0          0          0          1
elapsed time: 0.0020441s
icp has converged:1 score: 0.0335167
trans matrix:
  0.997864  0.0289145  0.0585766 -0.0393872
-0.0322259   0.997888  0.0563997  0.0977152
-0.0568221 -0.0581669   0.996688  0.0830583
         0          0          0          1
trans xyz:
0.134158 quat -0.0286688 0.028877 -0.0152996 0.999055  angle in rad/deg): 0.086973,4.98571deg
icp_main2 pose_observation_in_submap->trans -0.0393872 quat -0.0286688
icp_match: pose_observation_in_submap.translation: { t: [-0.0393872, 0.0977152, 0.0830583], q: [0.999055, -0.0286688, 0.028877, -0.0152996] }
scan_point_cloud,hgrid_point_cloud, 0x5568f6fb7720 0x5568f6fc5570
show aligned cloud 0 0
scan_2_pcd #points: 318 /home/student/Documents/cartographer/test/scan_hrt_edge_6.pcd
scan_2_pcd #points: 159 /home/student/Documents/cartographer/test/scan_hrt_6.pcd
scan_2_pcd #points: 318 /home/student/Documents/cartographer/test/scan_hrt_voxeledge_6.pcd

****** insert into submap******
scan_2_pcd #points: 595 range_data.pcd
insert data: returns/misses.size(): 595 0
hgrid_info: HybridGrid #points: 4060
hgrid_info: HybridGrid #points: 4301
hgrid_info: HybridGrid #points: 538
hgrid_info: HybridGrid #points: 548
submap info: HybridGrid #points/hit#/miss#: 4301 1842 2459 max/min prob: 0.76923 0.440255
local_pose, local_to_map,  tracking_to_local  -0.0393872 0.0977152 0.0830583, 0 0 0, -0.0127397 0.117409 0.0834018
SensorBridge::HandleRangefinder trajectory_builder_: N12cartographer7mapping25CollatedTrajectoryBuilderE
******LocalTrajectoryBuilder3D::AddAccumulatedRangeData
PoseExtrapolator::GetImuData imu_data_.empty() 0
GetImuData front time: 637720997681937978 imu_data_.size(): 1
GetImuData back time: 637720997681937978
scan_2_pcd #points: 583 /home/student/Documents/cartographer/test/filtered_range_data_in_tracking_7.pcd

******   high_resolution_adaptive_voxel_filter_options.min_num_points ******
150scanmatch_mode: 4
mode 3/4, use_edge_filter(): 0
*******ScanMatch_icp call
*******ScanMatch_icp first entry


submap_2_pcd: { t: [0, 0, 0], q: [1, 6.93889e-18, -2.32934e-21, -2.1684e-19] }
***** hybridgrid_2_pcd probthresh: 0.5
	submap info: HybridGrid #points/hit#/miss#: 4301 1842 2459 max/min prob: 0.76923 0.440255
	submap # points: 1842 submap_test_h_7.pcd

***** hybridgrid_2_pcd probthresh: 0.5
	submap info: HybridGrid #points/hit#/miss#: 548 321 227 max/min prob: 0.76923 0.440255
	submap # points: 321 submap_test_l_7.pcd
scan_2_pcd #points: 152 /tmp/tst.pcd
icp_match:  scan_point_cloud, 0x5568f6fc5610 icp_match: scan_point_cloud.width 152
icp_match: hgrid_point_cloud, 0x5568f6fb7720 0x5568f6fb7720
icp_match: hgrid_point_cloud/scan_point_cloud w/h: 1842 1 152 1
icp_match: initial_ceres_pose.translation: -0.0390662 0.0979524 0.0830624
using ndt verison
init matrix:
  0.997864  0.0289363   0.058575 -0.0390662
-0.0322476   0.997887  0.0563993  0.0979524
-0.0568193 -0.0581677   0.996689  0.0830624
         0          0          0          1
elapsed time: 0.00166625s
icp has converged:1 score: 0.0325769
trans matrix:
  0.998156  0.0313694  0.0519656 -0.0852525
-0.0346532   0.997377  0.0635442  0.0930318
-0.0498359 -0.0652278   0.996625  0.0809801
         0          0          0          1
trans xyz:
0.149936 quat -0.0322246 0.0254754 -0.0165219 0.999019  angle in rad/deg): 0.0885815,5.07792deg
icp_main2 pose_observation_in_submap->trans -0.0852525 quat -0.0322246
icp_match: pose_observation_in_submap.translation: { t: [-0.0852525, 0.0930318, 0.0809801], q: [0.999019, -0.0322246, 0.0254754, -0.0165219] }
scan_point_cloud,hgrid_point_cloud, 0x5568f6fc5610 0x5568f6fb7720
show aligned cloud 0 0
scan_2_pcd #points: 340 /home/student/Documents/cartographer/test/scan_hrt_edge_7.pcd
scan_2_pcd #points: 152 /home/student/Documents/cartographer/test/scan_hrt_7.pcd
scan_2_pcd #points: 340 /home/student/Documents/cartographer/test/scan_hrt_voxeledge_7.pcd

****** insert into submap******
scan_2_pcd #points: 583 range_data.pcd
insert data: returns/misses.size(): 583 0
hgrid_info: HybridGrid #points: 4413
hgrid_info: HybridGrid #points: 4589
hgrid_info: HybridGrid #points: 558
hgrid_info: HybridGrid #points: 570
submap info: HybridGrid #points/hit#/miss#: 4589 1992 2597 max/min prob: 0.802924 0.430416
local_pose, local_to_map,  tracking_to_local  -0.0852525 0.0930318 0.0809801, 0 0 0, -4.72063 -0.380296 -0.129044
local_pose, local_to_map,  tracking_to_local  -0.0852525 0.0930318 0.0809801, 0 0 0, -4.84574 -0.393071 -0.134712
local_pose, local_to_map,  tracking_to_local  -0.0852525 0.0930318 0.0809801, 0 0 0, -4.97099 -0.40586 -0.140387
local_pose, local_to_map,  tracking_to_local  -0.0852525 0.0930318 0.0809801, 0 0 0, -5.09594 -0.418619 -0.146049
local_pose, local_to_map,  tracking_to_local  -0.0852525 0.0930318 0.0809801, 0 0 0, -5.22155 -0.431446 -0.15174
local_pose, local_to_map,  tracking_to_local  -0.0852525 0.0930318 0.0809801, 0 0 0, -5.34652 -0.444207 -0.157402
local_pose, local_to_map,  tracking_to_local  -0.0852525 0.0930318 0.0809801, 0 0 0, -5.47137 -0.456955 -0.163059
local_pose, local_to_map,  tracking_to_local  -0.0852525 0.0930318 0.0809801, 0 0 0, -5.59657 -0.46974 -0.168732
local_pose, local_to_map,  tracking_to_local  -0.0852525 0.0930318 0.0809801, 0 0 0, -5.72159 -0.482506 -0.174396
local_pose, local_to_map,  tracking_to_local  -0.0852525 0.0930318 0.0809801, 0 0 0, -5.84669 -0.49528 -0.180064
local_pose, local_to_map,  tracking_to_local  -0.0852525 0.0930318 0.0809801, 0 0 0, -5.97177 -0.508052 -0.185732
local_pose, local_to_map,  tracking_to_local  -0.0852525 0.0930318 0.0809801, 0 0 0, -6.09709 -0.520849 -0.19141
local_pose, local_to_map,  tracking_to_local  -0.0852525 0.0930318 0.0809801, 0 0 0, -6.22204 -0.533608 -0.197071
SensorBridge::HandleRangefinder trajectory_builder_: N12cartographer7mapping25CollatedTrajectoryBuilderE
PoseGraph3D::AddImuData: -0.343298 -0.70973 10.021 time: 637720997682137978
PoseGraph3D::AddImuData: -0.357289 -0.68696 10.058 time: 637720997682377978
PoseGraph3D::AddImuData: -0.367236 -0.670138 10.0186 time: 637720997682577978
PoseGraph3D::AddImuData: -0.351791 -0.676588 9.99441 time: 637720997682777978
PoseGraph3D::AddImuData: -0.342551 -0.712895 10.0307 time: 637720997682977901
PoseGraph3D::AddImuData: -0.340003 -0.710214 10.0143 time: 637720997683177901
PoseGraph3D::AddImuData: -0.326335 -0.68797 10.0045 time: 637720997683377901
PoseGraph3D::AddImuData: -0.315065 -0.729583 10.0101 time: 637720997683577901
PoseGraph3D::AddImuData: -0.347893 -0.701211 9.9926 time: 637720997683737901
PoseGraph3D::AddImuData: -0.333613 -0.71134 10.0106 time: 637720997683947816
PoseGraph3D::AddImuData: -0.365337 -0.711088 9.99286 time: 637720997684147816
PoseGraph3D::AddImuData: -0.336325 -0.720876 10.0446 time: 637720997684347816
PoseGraph3D::AddImuData: -0.346046 -0.721413 10.0613 time: 637720997684547816
******LocalTrajectoryBuilder3D::AddAccumulatedRangeData
PoseExtrapolator::GetImuData imu_data_.empty() 0
GetImuData front time: 637720997681937978 imu_data_.size(): 14
GetImuData back time: 637720997684547816
scan_2_pcd #points: 574 /home/student/Documents/cartographer/test/filtered_range_data_in_tracking_8.pcd

******   high_resolution_adaptive_voxel_filter_options.min_num_points ******
150scanmatch_mode: 4
mode 3/4, use_edge_filter(): 0
*******ScanMatch_icp call
*******ScanMatch_icp first entry


submap_2_pcd: { t: [0, 0, 0], q: [1, 6.93889e-18, -2.32934e-21, -2.1684e-19] }
***** hybridgrid_2_pcd probthresh: 0.5
	submap info: HybridGrid #points/hit#/miss#: 4589 1992 2597 max/min prob: 0.802924 0.430416
	submap # points: 1992 submap_test_h_8.pcd

***** hybridgrid_2_pcd probthresh: 0.5
	submap info: HybridGrid #points/hit#/miss#: 570 332 238 max/min prob: 0.802924 0.430416
	submap # points: 332 submap_test_l_8.pcd
scan_2_pcd #points: 157 /tmp/tst.pcd
icp_match:  scan_point_cloud, 0x5568f6fc5570 icp_match: scan_point_cloud.width 157
icp_match: hgrid_point_cloud, 0x5568f6fc5610 0x5568f6fc5610
icp_match: hgrid_point_cloud/scan_point_cloud w/h: 1992 1 157 1
icp_match: initial_ceres_pose.translation: -3.344 -0.239725 -0.0666702
using ndt verison
init matrix:
  0.998097  0.0329862   0.052098     -3.344
-0.0362771    0.99732  0.0635401  -0.239725
-0.0498624 -0.0653092   0.996619 -0.0666702
         0          0          0          1
elapsed time: 0.0191872s
icp has converged:1 score: 2.74034
trans matrix:
  0.982135  -0.186511 -0.0249728   -2.69143
  0.175835   0.862345   0.474808   0.266681
-0.0670217  -0.470717   0.879735  0.0305569
         0          0          0          1
trans xyz:
2.70478 quat -0.244977 0.0108945 0.0938808 0.964911  angle in rad/deg): 0.531381,30.4613deg
icp_main2 pose_observation_in_submap->trans -2.69143 quat -0.244977
icp_match: pose_observation_in_submap.translation: { t: [-2.69143, 0.266681, 0.0305569], q: [0.964911, -0.244977, 0.0108945, 0.0938808] }
scan_point_cloud,hgrid_point_cloud, 0x5568f6fc5570 0x5568f6fc5610
show aligned cloud 0 0
scan_2_pcd #points: 289 /home/student/Documents/cartographer/test/scan_hrt_edge_8.pcd
scan_2_pcd #points: 157 /home/student/Documents/cartographer/test/scan_hrt_8.pcd
scan_2_pcd #points: 289 /home/student/Documents/cartographer/test/scan_hrt_voxeledge_8.pcd

****** insert into submap******
scan_2_pcd #points: 574 range_data.pcd
insert data: returns/misses.size(): 574 0
hgrid_info: HybridGrid #points: 5077
hgrid_info: HybridGrid #points: 5873
hgrid_info: HybridGrid #points: 691
hgrid_info: HybridGrid #points: 824
submap info: HybridGrid #points/hit#/miss#: 5873 2486 3387 max/min prob: 0.802924 0.430416
local_pose, local_to_map,  tracking_to_local  -2.69143 0.266681 0.0305569, 0 0 0, -5.99279 0.48665 -0.0333165
SensorBridge::HandleRangefinder trajectory_builder_: N12cartographer7mapping25CollatedTrajectoryBuilderE
PoseGraph3D::AddImuData: -0.344373 -0.711033 10.0214 time: 637720997684787816
PoseGraph3D::AddImuData: -0.312171 -0.710921 9.99812 time: 637720997684947724
******LocalTrajectoryBuilder3D::AddAccumulatedRangeData
PoseExtrapolator::GetImuData imu_data_.empty() 0
GetImuData front time: 637720997684547816 imu_data_.size(): 3
GetImuData back time: 637720997684947724
scan_2_pcd #points: 589 /home/student/Documents/cartographer/test/filtered_range_data_in_tracking_9.pcd

******   high_resolution_adaptive_voxel_filter_options.min_num_points ******
150scanmatch_mode: 4
mode 3/4, use_edge_filter(): 0
*******ScanMatch_icp call
*******ScanMatch_icp first entry


submap_2_pcd: { t: [0, 0, 0], q: [1, 6.93889e-18, -2.32934e-21, -2.1684e-19] }
***** hybridgrid_2_pcd probthresh: 0.5
	submap info: HybridGrid #points/hit#/miss#: 5873 2486 3387 max/min prob: 0.802924 0.430416
	submap # points: 2486 submap_test_h_9.pcd

***** hybridgrid_2_pcd probthresh: 0.5
	submap info: HybridGrid #points/hit#/miss#: 824 458 366 max/min prob: 0.832759 0.420625
	submap # points: 458 submap_test_l_9.pcd
scan_2_pcd #points: 152 /tmp/tst.pcd
icp_match:  scan_point_cloud, 0x5568f6fb7720 icp_match: scan_point_cloud.width 152
icp_match: hgrid_point_cloud, 0x5568f6fc5570 0x5568f6fc5570
icp_match: hgrid_point_cloud/scan_point_cloud w/h: 2486 1 152 1
icp_match: initial_ceres_pose.translation: -3.02762 0.289081 0.0240524
using ndt verison
init matrix:
  0.982179  -0.186286 -0.0249507   -3.02762
  0.175629   0.862396   0.474793   0.289081
  -0.06693  -0.470713   0.879744  0.0240524
         0          0          0          1
elapsed time: 0.00324073s
icp has converged:1 score: 0.091191
trans matrix:
  0.981855  -0.182902 -0.0500745   -2.69824
  0.182745   0.842089   0.507435   0.231471
-0.0506437  -0.507378   0.860234  0.0509758
         0          0          0          1
trans xyz:
2.70863 quat -0.264354 0.00014828 0.0952494 0.959711  angle in rad/deg): 0.569652,32.6552deg
icp_main2 pose_observation_in_submap->trans -2.69824 quat -0.264354
icp_match: pose_observation_in_submap.translation: { t: [-2.69824, 0.231471, 0.0509758], q: [0.959711, -0.264354, 0.00014828, 0.0952494] }
scan_point_cloud,hgrid_point_cloud, 0x5568f6fb7720 0x5568f6fc5570

