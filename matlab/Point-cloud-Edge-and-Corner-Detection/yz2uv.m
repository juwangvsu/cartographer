 function [u,v]= yz2uv(z,y,zmin,zmax,ymin,ymax,w,h)
 %convert point x,y to image space u,v. w,h image space, not perspective
 %proj.
 if z>zmax | z<zmin
     disp("bad input z")
 end
 if y>ymax | y<ymin
     disp("bad input y")
 end
  u = floor(w*(z-zmin)/(zmax-zmin))+1;
  v = floor(h*(y-ymin)/(ymax-ymin))+1;
 end