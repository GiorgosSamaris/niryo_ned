%%3D-Trajectory
clear
clc
%% Inputs
xo=input("xo");
yo=input("yo");
zo=input("zo");

xf=input("xf");
yf=input("yf");
zf=input("zf");
n=1;
%{
while(Taylor Criterion)
n=n+1
step_x=(xf-xo)/(n-1);
step_y=(yf-yo)/(n-1);
step_z=(zf-zo)/(n-1);
points=zeros(n,3);
%%Contour Calculation
i=1;
while(i<=n)
points(i,1)=(xo+(i-1)*step_x)^2; %test function x(x_relative)=x_relative^2  We set the Functions                                                                                 
points(i,2)=(yo+(i-1)*step_y); %test function y(y_relative)=y_relative
points(i,3)=(zo+(i-1)*step_z); %test function z(z_relative)=z_relative
i=i+1;
end
%Your Part of the algorithm for inverse kinematics calculation, median
point from points, median point for inverse kimematic p_avg(q_avg)
plot3(points(:,1),points(:,2),points(:,3))
%} 