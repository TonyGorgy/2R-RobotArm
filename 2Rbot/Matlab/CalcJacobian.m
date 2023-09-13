function J = CalcJacobian(idx) 
global uLINK

jsize = length(idx);
target = uLINK(idx(end)).p; 
J = zeros(6,jsize);
for n=1:jsize
% absolute target position
    j = idx(n);
    mom = uLINK(j).mother;
    a = uLINK(mom).R * uLINK(j).a; % joint axis in world frame 
    J(:,n) = [cross(a, target - uLINK(j).p) ; a ];% sum all of the J
end