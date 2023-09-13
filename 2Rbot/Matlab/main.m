Initialization;

%--------------2Rbot PARAMETERS---------------------------------
total_time = 11;
t = linspace(0,11,1000);
xaxis_points=[0;1;3;6;7;9;11];
yaxis_points=[0,1.6,1.8,1.2,1,0.8,1.3;
              0, 0, 0, 0, 0, 0, 0;              %Velocity in each point
              0, 0, 0, 0, 0, 0, 0]';            %Acceleration in each point

%--------------Arm's rotation angle and end's initial position-- 
Rf.p=[0;0;1.5];
Rf.R=[1,0,0;
      0,1,0;
      0,0,1];

%--------------Communicate with VREP-----------------------------
if (clientID>-1)
   disp('Connected to remote API server');
   h = [0,0]; %get joint handle
   [r,h(1)] = sim.simxGetObjectHandle(clientID,'Joint_1',sim.simx_opmode_blocking);
   [r,h(2)] = sim.simxGetObjectHandle(clientID,'Joint_2',sim.simx_opmode_blocking);
   
   joint_pos1 = 0.25 * pi;
   joint_pos2 = 0.25 * pi;

%---------Generate smooth moving points--------------------------
polynomial5_interpolation = Polynomial5Interpolation('Polynomial 5', yaxis_points, xaxis_points);
polynomial5_trajectory = zeros(length(t), 3); % N x 3 array: position, velocity, acceleration
for i = 1:length(t)
    polynomial5_trajectory(i,:) = polynomial5_interpolation.getPosition(t(i));
end
p_target = [0.1*t',polynomial5_trajectory(:,1)];
%p_target is the position that arm's end goes to;

%-----------------------------MAIN-----------------------------
    for i = 1:length(t)
       y = p_target(i,1);
       z = p_target(i,2);
       Rf.p = [0;y;z];
       %generate inverseKinematics data of each links
       InverseKinematics(4,Rf);
       Joint_1_pos = uLINK(2).q;
       Joint_2_pos = uLINK(3).q;
       sim.simxSetJointTargetPosition(clientID,h(1),Joint_1_pos,sim.simx_opmode_blocking);
       sim.simxSetJointTargetPosition(clientID,h(2),Joint_2_pos,sim.simx_opmode_blocking);
       pause(0.001);
       
   end
  
else
   disp('Failed connecting to remote API server');
end

   sim.delete(); % call the destructor!
   disp('Program ended');