function [ G_proj ] = stereoJac( p_jc_c,params )
%STEREOJAC Jacobian of stereo projection, needs 3 x 1 input point 
%   Detailed explanation goes here

G_proj = zeros(4,3);
x = p_jc_c(1);
y = p_jc_c(2);
z = p_jc_c(3);
G_proj(1,:) = [params.f_u/z, 0, -params.f_u*x/(z^2)];
G_proj(2,:) = [0, params.f_v/z, -params.f_v*y/(z^2)];
G_proj(3,:) = [params.f_u/z, 0, -params.f_u*(x-params.b)/(z^2)];
G_proj(4,:) = [0, params.f_v/z, -params.f_v*y/(z^2)];

end

