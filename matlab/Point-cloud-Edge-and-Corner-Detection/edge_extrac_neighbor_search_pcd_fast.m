function edge_cloud = edge_extrac_neighbor_search_pcd_fast(fn, showfig, radius)
% fast search alg: 
% input point cloud: structured as WxHx3
%radius: abs dist threshold (meters) within which the points will be kept
%as edge, assume opening raw pcd file that contain WxHx3 point, some points
%isnana(), this is different from the ply file which is already filtered to
%remove nan, and only contain valid points

global zmin zmax xmin xmax ymin ymax XX2_edge width height fig1 fig2 fig_edge_pc fig_edge fig_edge2 map
%ptCloud = pcread('C:\Users\Ju Wang\Documents\turtlebot3_imuodompt2_3\mapdata_37.pcd')
ptCloud = pcread(fn)

sz=size(ptCloud.Location)


XX = zeros(200,320);
xvec = ptCloud.Location(:,:,1);
yvec = ptCloud.Location(:,:,2);
zvec = ptCloud.Location(:,:,3);
xmin=min(xvec);
xmax=max(xvec);
ymin=min(yvec);
ymax=max(yvec);
zmin=min(zvec);
zmax=max(zvec);
width=320;
height = 200;
neighbor_cnt = zeros(sz(1),sz(2));
px_dist= pixel_dist(ptCloud);
s_w = floor(radius/px_dist/5)
n_cnt_threshold=0 % this estimate the number of neighbors for a inner point: currently as maximum ncnt we encounted
for i = 1: sz(1)
    for j=1:sz(2)
    pt1 = [xvec(i,j) yvec(i,j) zvec(i,j)];
    for k  = 1: s_w
        for h=1:s_w
            ii = floor(i+k-s_w/2);
            jj= floor(j+h-s_w/2);
            if ii>0 & ii < sz(1) & jj >0 & jj < sz(2) % check range
                G2=[xvec(ii,jj) yvec(ii,jj) zvec(ii,jj)];
                if ~any(isnan(G2)) % check it is a valid point
                    tmpdist = sqrt(sum((pt1 - G2) .^ 2));
                    if tmpdist< radius
                        neighbor_cnt(i,j) = neighbor_cnt(i,j) +1;
                        if neighbor_cnt(i,j) >  n_cnt_threshold
                            n_cnt_threshold=neighbor_cnt(i,j);
                        end
                    end
                end
            end %end check range
        end
    end
    end
end
edgepointcnt=0;
for i = 1: sz(1)
    for j=1:sz(2)
    if neighbor_cnt(i,j) < 0.7* n_cnt_threshold % this a likely to remove some points that hight curve area, might want to include surface norm for check
                 edgepointcnt = edgepointcnt+1;
                 ptlist(edgepointcnt,:)= [xvec(i,j), yvec(i,j), zvec(i,j)];
    end
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