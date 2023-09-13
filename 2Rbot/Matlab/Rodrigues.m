function [rotation_matrix] = Rodrigues(a ,theta)
%   Convert a rotation vector and rotation theta to a rotation matrix.
    a = a';
    I = eye(3);
    tmp_matrix_a = [
        0 -a(3) a(2); ...
        a(3) 0 -a(1); ...
        -a(2) a(1) 0];
    rotation_matrix = I + (1 - cos(theta)) * ...
    (tmp_matrix_a * tmp_matrix_a) + sin(theta) * tmp_matrix_a;
end