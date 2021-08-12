function drawResult(plotFn, b, bfDim,D)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Draw the trajectory and uncertainty ellipses
%
% Input:
%   plotFn: function handle which sets line data
%   b: the beliefs to plot
%   stDim: robot state space dimension
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
L = size(b,2);
if bfDim == 20
    stDim = 5;
elseif bfDim == 9
    stDim = 3;
end

itp = round(linspace(1,L,ceil(L/3))); % indexes to plot

x = b(1:stDim,:);

 pointsToPlot = [x(1,:) NaN;x(2,:) NaN];
% pointsToPlot = [];

Ne = 50;% number of points in ellipse drawing
% s  = 0.1; % size of the box around robot center
inc= 2*pi/Ne;
phi= 0:inc:2*pi;
sigmaScale = 3;

% plot box around robot center
% if stDim == 5   
%     itp = round(linspace(1,L,ceil(L/5))); % indexes to plot means    
%     for i = itp
%             plotBoxes(x(:,i),0.3);
%     end
% elseif  stDim == 3
%     for i = itp
%             plotDiscs(x(:,i),0.5)
%     end
% end

% get covariances
for i = itp
    
    vecP = D*b(stDim+1:end,i);
    Sigma = reshape(vecP,stDim,stDim); % Covariance Matrix

    ptemp = make_ellipse(x(1:2,i),Sigma(1:2,1:2), sigmaScale, phi);
    
    if isempty(ptemp) == 0
        pointsToPlot = [pointsToPlot ptemp];
    end
end

plotFn(pointsToPlot);


end

function p= make_ellipse(x,P,s, phi)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% make a single 2-D ellipse of s-sigmas
% over phi angle intervals
%
% Input:
%   x: mean
%   P: covariance matrix
%   s: confidence bound (1-sigma, 2-sigma etc)
%   phi: angles from o to 2*pi
%
% Output:
%   p: the points on perimiter of ellipse
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if s == 2 % 95% confidence
    chi2 = 5.991;
elseif s == 3 % 99% confidence
    chi2 = 9.210;
else
    error('Unknown confidence bound for drawing error ellipse');
end

magnify = 1.0; % scale up drawing

C = cholcov(P);

p = [];

if isempty(C) == 0
    a = C'*magnify*sqrt(chi2)*[cos(phi); sin(phi)];
    
    p=[a(1,:)+x(1) NaN;a(2,:)+x(2) NaN];
end

end

function plotBoxes(x,s)
    x_c = x(1);
    y_c = x(2);
    th = x(3);
    
    pts = 2.*[-0.1/2, 0.1,-0.1/2,-0.1/2, 0.1,-0.1/2;
            -0.1/2,0,0.1/2,-0.1/2,0,0.1/2;
            0,0,0,s/2,s/2,s/2];
    
    rot = axang2rotm([0,0,1,th]);
    pts = rot*pts;
    pts = pts + [x_c;y_c;0];
%     patch(pts(1,1:4),pts(2,1:4),pts(3,1:4));
%     patch(pts(1,5:8),pts(2,5:),pts(3,1:4));
    Faces = [4,5,6,NaN;
             1,2,5,4;
             2,3,6,5;
             1,3,6,4];
    p = patch('Faces',Faces,'Vertices',pts','FaceColor','cyan');
    p.Annotation.LegendInformation.IconDisplayStyle = 'off';
                    
    
end

function plotDiscs(x,s)
    robotDisk = [0.1.*cos(linspace(0,2*pi,50));...
                    0.1.*sin(linspace(0,2*pi,50));
                    s*ones(1,50)];
    rh = fill3(x(1) + robotDisk(1,:),x(2) + robotDisk(2,:),robotDisk(3,:),'b');
    rh.Annotation.LegendInformation.IconDisplayStyle = 'off';
end
