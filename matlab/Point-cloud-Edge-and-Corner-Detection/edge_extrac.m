function edge_cloud = edge_extrac(fn, showfig, radius)
%radius: abs dist threshold (meters) within which the points will be kept
%as edge

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
width=320
height = 200
[uu,vv] = xy2uv(xvec,yvec,xmin,xmax,ymin,ymax,width, height);
for i = 1: length(vv)
    XX(vv(i),uu(i))=zvec(i)*100;
end

XX2=imgaussfilt(XX,2); 
XX2_edge = edge(XX2,'Prewitt'); % the edge obtained here will not contain left board area. this is unintented, but ok

X= ptCloud.Location(:,:,:);
edge_cloud = calc_edge_cloud(xvec, yvec, zvec, radius);

% now deal with yz plan
[uu,vv] = yz2uv(zvec,yvec,zmin,zmax,ymin,ymax,width, height);
for i = 1: length(vv)
    XX(vv(i),uu(i))=xvec(i)*100;
end

XX2=imgaussfilt(XX,2); 
XX2_edge = edge(XX2,'Prewitt'); % the edge obtained here will not contain left board area. this is unintented, but ok
edge_cloud2 = calc_edge_cloud_yzplane(xvec, yvec, zvec, radius);

if (showfig)
    figure(fig1)
    pcshow(ptCloud)
    figure(fig2);
    imshow(XX,map)
    figure(fig_edge);
    imshow(XX2_edge)
    figure(fig_edge2);
    imshowpair(XX2,XX2_edge)
    title('gaussian filter -> edge extract')
    figure(fig_edge_pc);
    pcshow(edge_cloud)
    title('edge cloud')
    figure
    pcshow(edge_cloud2)
    title('edge cloud yzplane')
    
end

end