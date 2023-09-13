function ForwardKinematics(j)
global uLINK
if j == 0 return;end
if j~=1
    i = uLINK(j).mother;
    uLINK(j).p = uLINK(i).R * uLINK(j).b + uLINK(i).p; 
    % p2 = p1 + Rotation_Matrix * 在1坐标系里2的位置 1p2
    uLINK(j).R = uLINK(i).R * Rodrigues(uLINK(j).a , uLINK(j).q);
    
end
ForwardKinematics(uLINK(j).sister);
ForwardKinematics(uLINK(j).child);