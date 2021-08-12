function [FIF] = generateFIF(om,map,v_alpha, ds, saveFile)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generates Fisher Information Field based on given observation model and
% map
% Based on "Fisher Information Field: an Efficient and Differentiable Map for
% Perception-aware Planning" by Zhang and Scaramuzza
%
% Input:
%   om: Observation Model
%   map: map containing landmark locations
%   v_alpha: visibility value at FoV edge to fit visibility approximation
%   ds: spatial discretization size for FIF
%
% Outputs:
%   FIF: 3D Array for 2D positional world. FIF(i,j,:) will retrieve the 
%        vectorized form of the information matrix at x = i, y = j position
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    name = 'FIF_3D1_v0_5_n_0_01.mat';
    om.set_vis_coeffs(v_alpha);
    I = map.bounds(1,2) - map.bounds(1,1);
    J = map.bounds(2,2) - map.bounds(2,1);
    FIF = zeros(I/ds,J/ds,om.stDim^2*om.N_v);
    
    x_grid = linspace(map.bounds(1,1),map.bounds(1,2),I/ds + 1);
    y_grid = linspace(map.bounds(2,1),map.bounds(2,2),J/ds + 1);
    
    for i = 1:length(x_grid)
        for j = 1:length(y_grid)
            p_ci_i = [x_grid(i);y_grid(j)];
            C_I_ij = om.compute_FIM_pos(p_ci_i);
            FIF(i,j,:) = C_I_ij(:);
        end
    end
    
    
    
    if saveFile == 1
        FIF_name = strcat('./FIF/',name);
        save(FIF_name,'FIF');
    end
end

