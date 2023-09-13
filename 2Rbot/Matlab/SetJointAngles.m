function SetJointAngles(idx, q) 
global uLINK
q=q*pi/180;
for n=1:length(idx)
      j = idx(n);
      uLINK(j).q = q(n); 
end
ForwardKinematics(1);