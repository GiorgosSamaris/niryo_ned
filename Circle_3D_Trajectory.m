%% Algorithm For X-Y Plane Choice Of  polygon Num
clear
clc

%% Inputs
R_circle=input("R");
%Vector for Center of Circle
PC_x=input("x center");
PC_y=input("y center");
PC_z=input("z center");
PC=[PC_x,PC_y,PC_z];

%Vector for Point 1                             %Point 1&2 will define the Plane of the circle these                                                    
P1_x=input("x P1");                             %points can be within the area of the circle,outside,
P1_y=input("y P1");                             %or on the circular contour
P1_z=input("z P1");
P1=[P1_x,P1_y,P1_z];

%Vector for Point 2
P2_x=input("x P2");
P2_y=input("y P2");
P2_z=input("z P2");
P2=[P2_x,P2_y,P2_z];

%% Creation of Plane and local coordiate system
a=P1-PC;
b=P2-PC;
plane_vec=[a(2)*b(3)-a(3)*b(2),a(3)*b(1)-a(1)*b(3),a(1)*b(2)-a(2)*b(1)];%((P1-PC)x(P2-PC));
unity_plane_vec=plane_vec/(sqrt(plane_vec(1)^2+plane_vec(2)^2+plane_vec(3)^2));
local_x_axis=a./(sqrt(a(1)^2+a(2)^2+a(3)^2));
a=unity_plane_vec;
b=local_x_axis;
local_y_axis=[a(2)*b(3)-a(3)*b(2),a(3)*b(1)-a(1)*b(3),a(1)*b(2)-a(2)*b(1)];                 %unity_plane_vec x local_x_axis;

%% Creation of Circle in Plane 
n_poly=2;
%%Use of Taylor_Criterion to choose number of polygons
%{
while(Taylor_Criterion)
    n_poly=n_poly+1;
    theta_turn=2*3.14/n_poly;
    points=zeros(2);
    points_avg=[0,0]
    points(1,1)=R;
    points(1,2)=0;
    points(2,1)=R_circle*cos((i-1)*theta_turn_final);
    points(2,2)=R_circle*sin((i-1)*theta_turn_final);
    %Your Part of the algorithm for inverse kinematics calculation, median
    point from points, median point for inverse kimematic p_avg(q_avg)

    n_poly_final=n_poly
%}
n_poly_final=82;
theta_turn_final=2*3.14/n_poly_final;
local_points=zeros(n_poly_final,3);
for i=1:n_poly_final
    local_points(i,1)=R_circle*cos((i-1)*theta_turn_final); 
    local_points(i,2)=R_circle*sin((i-1)*theta_turn_final); 
end
%% Transformation to Local Coordinate system Coordinates(DOT PRODUCT BETWEEN VECTORS OF POINTS AND UNITY VECTORS
local_trans=zeros(n_poly_final,3);
for i=1:n_poly_final
    local_trans(i,1)=local_points(i,1)*local_x_axis(1)+local_points(i,2)*local_x_axis(2)+local_points(i,3)*local_x_axis(3);
    local_trans(i,2)=local_points(i,1)*local_y_axis(1)+local_points(i,2)*local_y_axis(2)+local_points(i,3)*local_y_axis(1);
    local_trans(i,3)=local_points(i,1)*unity_plane_vec(1)+local_points(i,2)*unity_plane_vec(2)+local_points(i,3)*unity_plane_vec(3);
end
%% Transformation to General Coordinate system
Trans_vectors=zeros(n_poly_final,3);
for i=1:n_poly_final
    Trans_vectors(i,:)=PC; 
end
global_points=Trans_vectors+local_trans;
plot3(global_points(:,1),global_points(:,2),global_points(:,3))

%{ 
Circlular movement
for i=1:n_poly_final-1
 initial_point=[points_points(i,1),points_points(i,2)
 final_point=[points_points(i+1,1),points_points(i+1,2)]
 %Your Part of the algorithm for traversing from one point to another
%}
