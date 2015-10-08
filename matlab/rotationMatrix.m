
%ROTATIONMATRIX get rotation matrix to rotate about an axis v, angle given by
% two vectors v_1 and v_2
%   Detailed explanation goes here

function [ R ] = rotationMatrix( angle_v_1, angle_v_2, axis_v )

    % ensure that angle vectors are unit vectors
    angle_v_1 = angle_v_1 / norm(angle_v_1);
    angle_v_2 = angle_v_2 / norm(angle_v_2);

    cp = cross(angle_v_1, angle_v_2);
    sin_angle = norm(cp);
    cos_angle = dot(angle_v_1, angle_v_2);

    axis_cross = [ 0            -axis_v(3)  axis_v(2); ...
                   axis_v(3)    0           -axis_v(1); ...
                   -axis_v(2)   axis_v(1)   0];
               
    axis_tensor = [ axis_v(1)*axis_v(1), axis_v(1)*axis_v(2), axis_v(1)*axis_v(3); ...
                    axis_v(2)*axis_v(1), axis_v(2)*axis_v(2), axis_v(2)*axis_v(3); ...
                    axis_v(3)*axis_v(1), axis_v(3)*axis_v(2), axis_v(3)*axis_v(3); ];
                
    R = (eye(3) * cos_angle + sin_angle * axis_cross + (1 - cos_angle) * axis_tensor)';
end

