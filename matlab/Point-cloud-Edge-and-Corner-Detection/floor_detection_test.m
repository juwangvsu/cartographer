fn="/home/student/Documents/cartographer/test/turtlebot3_imuodompt2_3/edge_fastpcd_0.1/mapdata_19.pcd"
ptCloud = pcread(fn)
rem_cloud = floor_detection(ptCloud,false)
figure
pcshow(rem_cloud)
title("remaining cloud, floor removed")