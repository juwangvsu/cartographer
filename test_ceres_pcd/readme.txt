---------------1/13/22 -------------------------------------
cartographer/test_ceres_pcd
	1-10-22/  hgrid no miss point
	1-6-22/   hgrid contain miss data point	
        scan_test_h.pcd
        submap_test_h.pcd
        readme.txt
	sweep3.ods		costfun plot sweeping init pos, see cartographer scanmatch test example.
			cartographer/build$ ./cartographer.mapping.internal.3d.scan_matching.ceres_scan_matcher_3d_test

two snap shot pcd file:
pcl_viewer  -fc 0,0,255 scan_test_h.pcd  -fc 0,255,0 submap_test_h.pcd


************initial pose (local frame): 
initial pose (local frame): 
{ t: [0.37273, 0.178726, -0.228209], q: [1, 2.61099e-05, 0.000306306, -1.56989e-05] }

result
	{ t: [0.429205, 0.15951, -0.201888], q: [1, 1.94222e-05, 0.00030947, -6.88515e-05] }

Initial                          3.303322e+00
Final                            3.170857e+00
Change                           1.324653e-01
-----------------------------------------------------------------------------
~/Documents/hdl_graph_slam/ndt/build/icp_example -pst=1 -tf=scan_test_h.pcd -if=submap_test_h.pcd -yawg=0.1 -xg=0 -rsl=0.02 -md=gicp -ms=-1
	-tf: target frame, -if: reference frame, result translation is to move tf to if

~/Documents/hdl_graph_slam/ndt/build/icp_example -pst=1 -if=../test/scan_hrt_30.pcd -tf=../test/scan_hrt_0.pcd -yawg=0.1 -xg=0 -rsl=0.02 -md=gicp -ms=-1

pcl_viewer  -fc 0,255,0 scan_test_h.pcd  -fc 255,0,0 submap_test_h.pcd
