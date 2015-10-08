%DISTORT Summary of this function goes here
%
% The distortion is calculated using following formula
%  u = a * (1 + kc(1)*r^2 + kc(2)*r^4 + kc(5)*r^6) + 2*kc(3)*a*b + kc(4)*(r^2 + 2*a^2)
%  v = b * (1 + kc(1)*r^2 + kc(2)*r^4 + kc(5)*r^6) + kc(3)*(r^2 + 2*b^2) + 2*kc(4)*a*b
% where u, v are the distorted point coordinates and a,b the original,
% ideal coordinates. r^2 = a^2 * b^2
%
% Then focal length, principal point and skew are taken into account
%
% Arguments:
%           p           - 2D Points on imageplane
%           image_dim   - 1x2 vector containing the width and height of the image
%           fc          - 1x2 vector containing the focal length of the camera
%           kc          - 1x6 vector containing the distortion parameters
%           cc          - 1x2 vector containing the center offset of the camera
%           alpha_c     - skew of the camera

function [ p_distorted ] = distort(p, image_dim, fc, kc, cc, alpha_c)
    p_norm(1,1) = (p(1)*2 - image_dim(1)) / image_dim(1);
    p_norm(2,1) = (p(2)*2 - image_dim(2)) / image_dim(2);

    r_2 = p_norm(1)^2 + p_norm(2)^2;
    radial_dist_factor = 1 + kc(1)*r_2 + kc(2)*r_2*r_2 + kc(5)*r_2*r_2*r_2;
    
    u = p_norm(1) * radial_dist_factor + 2*kc(3)*p_norm(1)*p_norm(2) + kc(4)*(r_2 + 2*p_norm(1)^2);
    v = p_norm(2) * radial_dist_factor + kc(3)*(r_2 + 2*p_norm(2)^2) + 2*kc(4)*p_norm(1)*p_norm(2);

    p_distorted(1,1) = (u + 1) * image_dim(1) / 2;
    p_distorted(2,1) = (v + 1) * image_dim(2) / 2;
end

