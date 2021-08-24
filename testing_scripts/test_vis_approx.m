clear all
close all
clc

ks = 20;
FoV = deg2rad(60);
th = linspace(-pi,pi,500);
y = 1./(1 + exp(-ks.*(cos(th) - cos(FoV/2))));
plot(rad2deg(th),y);
hold on;

p_vis = 0.5.*(cos(pi.*th/(FoV/2)) + 1);
p_vis(abs(th)>FoV/2) = 0;
plot(rad2deg(th),p_vis,'g');

modelfunc = @(A,x) A.'*[ones(1,length(x)); cos(x); (cos(x)).^2; cos(x).^3;cos(x).^4;cos(x).^5];
beta0 = [-1.2321;0.5;1.7321;0;0;0];
beta = nlinfit(th,y,modelfunc,beta0);
% beta = nlinfit(th,p_vis,modelfunc,beta0);
plot(rad2deg(th),modelfunc(beta,th),'r-');



% syms p1 p2 p3 z1 z2 z3 k0 k1 k2 k3 k4;
% dot = z1*p1 + z2*p2 + z3*p3;
% vis = k4*dot.^4 + k3*dot.^3 + k2*dot.^2 + k1*dot + k0;
% expand(vis)
% 
% v_r = [k4*z1^4, 4*k4*z1^3*z2, 4*k4*z1^3*z3, 6*k4*z1^2*z2^2, 12*k4*z1^2*z2*z3,...
%         6*k4*z1^2*z3^2, 4*k4*z1*z2^3, 12*k4*z1*z2^2*z3, 12*k4*z1*z2*z3^2, ...
%         4*k4*z1*z3^3, k4*z2^4, 4*k4*z2^3*z3, 6*k4*z2^2*z3^2, 4*k4*z2*z3^3,...
%         k4*z3^4, k3*z1^3,3*k3*z1^2*z2, 3*k3*z1^2*z3, 3*k3*z1*z2^2, 6*k3*z1*z2*z3,...
%         3*k3*z1*z3^2, k3*z2^3, 3*k3*z2^2*z3, 3*k3*z2*z3^2, k3*z3^3, ...
%         k2*z1^2, 2*k2*z1*z2, 2*k2*z1*z3, k2*z2^2, 2*k2*z2*z3, k2*z3^2,...
%         k1*z1, k1*z2, k1*z3, k0];
%     
% v_p = [p1^4, p1^3*p2, p1^3*p3, p1^2*p2^2, p1^2*p2*p3, ...
%         p1^2*p3^2,p1*p2^3,p1*p2^2*p3, p1*p2*p3^2,...
%         p1*p3^3, p2^4, p2^3*p3, p2^2*p3^2, p2*p3^3,...
%         p3^4, p1^3, p1^2*p2, p1^2*p3, p1*p2^2, p1*p2*p3,...
%         p1*p3^2, p2^3, p2^2*p3, p2*p3^2, p3^3,...
%         p1^2, p1*p2, p1*p3, p2^2, p2*p3, p3^2,...
%         p1, p2, p3, 1];
        
        