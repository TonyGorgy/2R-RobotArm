clc;
clear;
load('new_para.mat')

%---------parameter settings-----
total_time = 11;
t = linspace(0,11,1000);
xaxis_points=[0;1;3;6;7;9;11];
yaxis_points=[0,1.6,1.8,1.2,1,0.8,1.3;
              0, 0, 0, 0, 0, 0, 0;
              0, 0, 0, 0, 0, 0, 0]';

%---------arm's rotation angle and end's initial position-- 
Rf.p=[0;1.00;1.30];
Rf.R=[1,0,0;0,1,0;0,0,1];

%---------Generate smooth moving points----
polynomial5_interpolation = Polynomial5Interpolation('Polynomial 5', yaxis_points, xaxis_points);
polynomial5_trajectory = zeros(length(t), 3); % N x 3 array: position, velocity, acceleration

for i = 1:length(t)
    polynomial5_trajectory(i,:) = polynomial5_interpolation.getPosition(t(i));
end

p_target = [0.1*t',polynomial5_trajectory(:,1)];
%p_target is the position that arm's end goes to;

%---------Draw the graph-----
plot(0.1*xaxis_points,yaxis_points(:,1),'ro');
axis([-1 2 0 3]);
hold on
for i = 1:length(t)
    y = p_target(i,1);
    z = p_target(i,2);
    Rf.p = [0;y;z];
    %generate inverseKinematics data of each links
    InverseKinematics(4,Rf);
    
end