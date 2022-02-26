function px_dist = pixel_dist(ptc)
sz= size(ptc.Location)
if length(sz)==2
    disp("only work for WxHx3 pcd")
    return
end

%initiate px_dist with the x-dist between first two points
px_dist = abs(ptc.Location(1,1,1)-ptc.Location(1,2,1));

for i=1:sz(1)-1
    for j=1:sz(2)-1
       % due to possibly perspective project, some points on the same row
       % might have the same x value (even x value suppose to lnearlly
       % increase, so we check that the increment must be > 0.003, an
       % empiracal value observed from the data, typical pixel-dist is
       % 0.007
        xdist1= abs(ptc.Location(i,j,1)-ptc.Location(i,j+1,1));
        if xdist1>0.003 
            px_dist = min(px_dist, xdist1);
        end
        ydist1= abs(ptc.Location(i,j,2)-ptc.Location(i+1,j,2));
        if ydist1>0.003 
            px_dist = min(px_dist, ydist1);
        end
    end
end
end