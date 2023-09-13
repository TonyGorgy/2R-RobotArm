function R = rpy2rot(phi,theta,psi)
% inputs: roll, pitch, yaw
% outputs: Rotation Matrix 
%           roll-pitch-yaw to Rotation Matrix   


R = [ cos(psi)*cos(theta),                              cos(theta)*sin(psi) ,                                -sin(theta);    ...
    -cos(phi)*sin(psi)+cos(psi)*sin(phi)*sin(theta),    cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi),      cos(theta)*sin(phi);...
    sin(psi)*sin(phi) + sin(theta)*cos(phi)*cos(psi),   -cos(psi)*sin(phi) + cos(phi)*sin(theta)*sin(psi),   cos(theta)*cos(phi)];
    
R = R';