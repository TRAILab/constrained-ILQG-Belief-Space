function [ y_j ] = stereo_proj( p_jc_v, params )
%STEREO_PROJ Stereo projection model
%   
x = p_jc_v(1);
y = p_jc_v(2);
z = p_jc_v(3);

y_j = zeros(4,1);

y_j(1) = params.f_u*x/z + params.c_u;
y_j(2) = params.f_v*y/z + params.c_v;
y_j(3) = params.f_u*(x-params.b)/z + params.c_u;
y_j(4) = params.f_v*y/z + params.c_v;


end

