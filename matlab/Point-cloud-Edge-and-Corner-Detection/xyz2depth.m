% extract edge point cloud from ply files in a folder 
% save in a specified folder
% ex parameters: edgeradius = 0.1
%     inputfolder
%     outputfolder
%     filename_prefix
% method 1: psudo proj to xy plane, image smoothing, edge detection, back
% track to point cloud

% method 2: direct neighbor search: for each 3D pt, check edgeradius and if
% # of neighbors < 70% of estimated full neighbor than it is a edge point,
% to be done.

% Ju Wang, 2022
close all
clear all
edgeradius =0.1
global  xmin xmax ymin ymax XX2_edge width height fig1 fig2 fig_edge_pc fig_edge fig_edge2 map
load clown.mat
fig1 = figure;
fig_edge_pc = figure;
fig2 = figure;
fig_edge = figure;
fig_edge2 = figure;
tic
%edge_extrac("C:\Users\Ju Wang\Documents\turtlebot3_imuodompt2_3\mapdata_9_.ply",1,0.1);

%edge_extrac_neighbor_search_pcd_fast("C:\Users\Ju Wang\Documents\turtlebot3_imuodompt2_3\mapdata_9.pcd",1,0.1);
% 2.34 secs

%edge_extrac_neighbor_search_pcd("C:\Users\Ju Wang\Documents\turtlebot3_imuodompt2_3\mapdata_9.pcd",1,0.1);
% 1000 secs

%edge_extrac_neighbor_search("C:\Users\Ju Wang\Documents\turtlebot3_imuodompt2_3\mapdata_9.pcd",1,0.1);
toc

%pause

% processing ply files in a folder 
if ispc
    inputfolder = "C:\Users\Ju Wang\Documents\turtlebot3_imuodompt2_3\";
    outputfolder = "C:\Users\Ju Wang\Documents\turtlebot3_imuodompt2_3\edge_fastpcd_0.1\"
elseif isunix
    inputfolder = "/home/student/Documents/cartographer/test/turtlebot3_imuodompt2_3/";
    outputfolder = "/home/student/Documents/cartographer/test/turtlebot3_imuodompt2_3/edge_fastpcd_0.1/"
end
mkdir(outputfolder)
filename_prefix="mapdata_*.pcd"
%filename_prefix="mapdata_*.ply"
plyfilelist = dir(inputfolder+filename_prefix)
tic
for i=1:length(plyfilelist)
   filen = inputfolder+plyfilelist(i,1).name
   outputfn = outputfolder+plyfilelist(i,1).name
   outputfn2 = outputfolder+plyfilelist(i,1).name+".ply"
   outputfn3 = outputfolder+plyfilelist(i,1).name+"_edge.pcd"
   outputfn4 = outputfolder+plyfilelist(i,1).name+"_edge.ply"
   %ptCloud = pcread(filen)
   %edge_cloud = edge_extrac(filen, 1,edgeradius)
   edge_cloud = edge_extrac_neighbor_search_pcd_fast(filen,1,edgeradius);
   rem_cloud = floor_detection(edge_cloud, false)
   pcwrite(edge_cloud, outputfn)
   pcwrite(edge_cloud, outputfn2)
   pcwrite(rem_cloud, outputfn3)
   pcwrite(rem_cloud, outputfn4)

end
toc

%[edge_points,edge_id,curve,surface_normal,cloud] = edgedetection(X)
%edge_cloud = pointCloud(edge_points);
%pcwrite(edge_cloud, 'output.ply')
%figure;
%pcshow(edge_cloud)
%xy2uv([1,1],[1,1],-3,1,-2,1,320,200)

