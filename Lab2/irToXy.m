function [x,y,th] = irToXy(i,r)

th = deg2rad(-5+(i-1));
if th > pi
    th = -(2*pi-th);
end

x =  r*cos(th);
y =  r*sin(th);


end