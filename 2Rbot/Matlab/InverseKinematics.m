function InverseKinematics(to, Target) 
%to and Target_one and target_R respectively
global uLINK
lambda = 0.9; 
ForwardKinematics(1); 
idx = FindRoute(to); %get target route
for n = 1:10
    J = CalcJacobian(idx);%计算J
    err = CalcVWerr(Target, uLINK(to)); %计算误差
    if norm(err) < 1E-6 return, end %如果误差小于10的-6次方，则终止
    dq = lambda * (J \ err);%计算修正值 lamda * inverse_J * err
    for nn=1:length(idx) % q = q + delta_q
        j = idx(nn);
        uLINK(j).q = uLINK(j).q + dq(nn); 
    end
    ForwardKinematics(1); %再次 forward_kinematics
    %plot_s(uLINK);
    %hold on
end
hold off