 function [u,v]= xy2uv(x,y,xmin,xmax,ymin,ymax,w,h)
 %convert point x,y to image space u,v. w,h image space, not perspective
 %proj.
 if x>xmax | x<xmin
     disp("bad input x")
 end
 if y>ymax | y<ymin
     disp("bad input y")
 end
  u = floor(w*(x-xmin)/(xmax-xmin))+1;
  v = floor(h*(y-ymin)/(ymax-ymin))+1;
 end