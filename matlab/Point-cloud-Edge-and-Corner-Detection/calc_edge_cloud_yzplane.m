function ed_cld = calc_edge_cloud_yzplane(xvec, yvec, zvec, radius)
%assume XX2_edge is the binary image of edge points
global zmin zmax ymin ymax XX2_edge width height
edgepointcnt=0;
ptlist=zeros(length(xvec),3);
disp('run calc_edge_cloud_yzplane ')
disp(length(zvec))
%pns: pixel_neighbor_size
XX2_index = zeros(size(XX2_edge))
pns= radius * (width/ (zmax-zmin) + height/ (ymax-ymin))/2
for i = 1: length(xvec)
    [uu,vv] = yz2uv(zvec(i),yvec(i),zmin,zmax,ymin,ymax,width, height);
    vv_left= floor(max(vv-pns,1));
    vv_right= floor(min(vv+pns,height));
    uu_left= floor(max(uu-pns,1));
    uu_right= floor(min(uu+pns,width));
    if sum(sum(XX2_edge(vv_left:vv_right,uu_left:uu_right)))<1
            continue;
    end
    if XX2_index(vv,uu)==0
        XX2_index(vv,uu) = i;
    end
    if xvec(i)< xvec(XX2_index(vv,uu))
        XX2_index(vv,uu) = i;
    end
end
for i = 1: length(xvec)
    flag=0
    %disp([xvec(i), yvec(i), zvec(i)])
    [uu,vv] = yz2uv(zvec(i),yvec(i),zmin,zmax,ymin,ymax,width, height);
    vv_left= floor(max(vv-pns,1));
    vv_right= floor(min(vv+pns,height));
    uu_left= floor(max(uu-pns,1));
    uu_right= floor(min(uu+pns,width));
%     vv_left= max(vv-pns,1);
%     vv_right= min(vv+pns,height);
%     uu_left= max(uu-pns,1);
%     uu_right= min(uu+pns,width);
    neighbor_square = XX2_index(vv_left:vv_right,uu_left:uu_right);
    [w,h]=size(neighbor_square);
    for ii=1:w
        for jj=1:h
            tmpindex=neighbor_square(ii,jj);
            if tmpindex==0
                continue;
            end
            G=[xvec(i) yvec(i) zvec(i)];
            G2=[xvec(tmpindex) yvec(tmpindex) zvec(tmpindex)];
            tmpdist = sqrt(sum((G - G2) .^ 2));
            if tmpdist< radius
                 edgepointcnt = edgepointcnt+1;
                 ptlist(edgepointcnt,:)= [xvec(i), yvec(i), zvec(i)];
                 flag=1;
                 break;
            end
        end
        if flag == 1
               break; %break from outer loop as well
        end
    end
    
%     if sum(sum(XX2_edge(vv_left:vv_right,uu_left:uu_right)))>1
%         edgepointcnt = edgepointcnt+1;
%         ptlist(edgepointcnt,:)= [xvec(i), yvec(i), zvec(i)];
%     end
end
ed_cld = pointCloud( ptlist(1:edgepointcnt,:));
end