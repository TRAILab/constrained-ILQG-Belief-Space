function [rh,lh] = drawFoV(fig, meas_model,x,rh,lh)
%DRAWFOV Draws the FoV of the camera given robot pose
%   Input: fig - figure handle
%          meas_model - ObservationModelBase object
%          x   - robot pose [x,y,theta]
%          rh,lh - handles for the FoV lines
    delete(rh);
    delete(lh);
    figure(fig);
    hold on;
    
    theta = x(3);
    r = 3;
    
    %draw max u line
    FoV = meas_model.FoV;
    rh = line([x(1);x(1) + r*cos(theta-(FoV/2))],[x(2);x(2) + r*sin(theta-(FoV/2))],'Color','red','LineStyle','--');
    lh = line([x(1);x(1) + r*cos(theta+(FoV/2))],[x(2);x(2) + r*sin(theta+(FoV/2))],'Color','red','LineStyle','--');
    set(get(get(rh,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
    set(get(get(lh,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
    
end

