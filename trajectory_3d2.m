%%3D-Trajectory
clear
clc
%% Inputs
to=input("to");
tf=input("tf");

n=2;
dis=0.1;
R_max=1e6;

while(R_max>dis)
    n=n+1;
    step_t=(tf-to)/(n-1);
    points=zeros(n,3);
    param_matrix=zeros(n,1);
    mid_points=zeros(n-1,3);
    mid_curves=zeros(n-1,3);
    R=zeros(n-1,1);
    for i=1:n
    %Parameters Calculation
    param_matrix(i,1)=to+(i-1)*step_t;
    %%Contour Calculation
    points(i,1)=(param_matrix(i,1))^2; %test function x(x_relative)=x_relative^2  We set the Functions                                                                                 
    points(i,2)=20*(param_matrix(i,1)); %test function y(y_relative)=y_relative
    points(i,3)=15*abs(param_matrix(i,1)); %test function z(z_relative)=z_relative
    end

    for i=1:n-1
        %Middle of line
        mid_points(i,1)=0.5*(points(i,1)+points(i+1,1));
        mid_points(i,2)=0.5*(points(i,2)+points(i+1,2));
        mid_points(i,3)=0.5*(points(i,3)+points(i+1,3));
        %Curve at mid_point  
        mid_curves(i,1)=(0.5*(param_matrix(i,1)+param_matrix(i+1,1)))^2; %mid_point of function
        mid_curves(i,2)=20*(0.5*(param_matrix(i,1)+param_matrix(i+1,1)));
        mid_curves(i,3)=15*abs(0.5*(param_matrix(i,1)+param_matrix(i+1,1)));
        R(i,1)=sqrt((mid_curves(i,1)-mid_points(i,1))^2+(mid_curves(i,2)-mid_points(i,2))^2+(mid_curves(i,3)-mid_points(i,3))^2);
        if i>2
            if R(i,1)> R(i-1,1)
             R_max= R(i,1); 
            end
        end
    end
end

plot3( points(:,1), points(:,2), points(:,3))
xlabel("x-axis")
ylabel("y-axis")
zlabel("z-axis")
