function edge_cloud = edge_extrac_neighbor_search(fn, showfig, radius)
% expect Wx1x3 point cloud, such as ply file
% radius: abs dist threshold (meters) within which the points will be kept
% as edge

global zmin zmax xmin xmax ymin ymax XX2_edge width height fig1 fig2 fig_edge_pc fig_edge fig_edge2 map
%ptCloud = pcread('C:\Users\Ju Wang\Documents\turtlebot3_imuodompt2_3\mapdata_37_.ply')
ptCloud = pcread(fn)

XX = zeros(200,320);
xvec = ptCloud.Location(:,1);
yvec = ptCloud.Location(:,2);
zvec = ptCloud.Location(:,3);
xmin=min(xvec);
xmax=max(xvec);
ymin=min(yvec);
ymax=max(yvec);
zmin=min(zvec);
zmax=max(zvec);
width=320;
height = 200;
neighbor_cnt = zeros(length(xvec),1);
n_cnt_threshold=0 % this estimate the number of neighbors for a inner point: currently as maximum ncnt we encounted
for i = 1: length(xvec)
    pt1 = [xvec(i) yvec(i) zvec(i)];
    for j  = 1: length(xvec)
        G2=[xvec(j) yvec(j) zvec(j)];
        tmpdist = sqrt(sum((pt1 - G2) .^ 2));
        if tmpdist< radius
             neighbor_cnt(i) = neighbor_cnt(i) +1;
             if neighbor_cnt(i) >  n_cnt_threshold
                 n_cnt_threshold=neighbor_cnt(i);
             end
        end
    end
end
edgepointcnt=0;
for i = 1: length(xvec)
    if neighbor_cnt(i) < 0.7* n_cnt_threshold % this a likely to remove some points that hight curve area, might want to include surface norm for check
                 edgepointcnt = edgepointcnt+1;
                 ptlist(edgepointcnt,:)= [xvec(i), yvec(i), zvec(i)];
    end
end
edge_cloud = pointCloud( ptlist(1:edgepointcnt,:));

if (showfig)
    figure(fig1)
    pcshow(ptCloud)

    figure(fig_edge_pc);
    pcshow(edge_cloud)
    title('edge cloud')
   
end

end