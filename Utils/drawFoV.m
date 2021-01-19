function [handles] = drawFoV(fig, meas_model,x,handles)
%DRAWFOV Draws the FoV of the camera given robot pose
%   Input: fig - figure handle
%          meas_model - ObservationModelBase object
%          x   - robot pose [x,y,theta]
%          rh,lh - handles for the FoV lines

    if ~isempty(handles)
        for i = 1:length(handles)
            delete(handles(i))
        end
    end
    
    figure(fig);
    hold on;
    
    theta = x(3);
    r = 0.7;
    
    %draw max u line
    FoV = meas_model.FoV;
    if meas_model.obsDim == 2
        handles = [line([x(1);x(1) + r*cos(theta-(FoV/2))],[x(2);x(2) + r*sin(theta-(FoV/2))],'Color','red','LineStyle','--');
                    line([x(1);x(1) + r*cos(theta+(FoV/2))],[x(2);x(2) + r*sin(theta+(FoV/2))],'Color','red','LineStyle','--');
                    ];
    else
        
        if length(x) == 3
            phi = 0;
            psi = meas_model.psi;
        else
            phi = x(4);
            psi = x(5);
        end
        
        pan = theta + phi;
        
        lpu = [ r*cos(pan+(FoV/2))*cos(- (FoV/2));
                r*sin(pan+(FoV/2))*cos(- (FoV/2));
               -r*sin(- (FoV/2))];
        rpu = [r*cos(pan-(FoV/2))*cos(- (FoV/2));
                r*sin(pan-(FoV/2))*cos(- (FoV/2));
               -r*sin(- (FoV/2))];
        lpl = [r*cos(pan+(FoV/2))*cos( (FoV/2));
               r*sin(pan+(FoV/2))*cos( (FoV/2));
               -r*sin( (FoV/2))];
        
        rpl = [ r*cos(pan-(FoV/2))*cos( (FoV/2));
                r*sin(pan-(FoV/2))*cos( (FoV/2));
               -r*sin(  (FoV/2))];
        rot = axang2rotm([-sin(pan),cos(pan),0,psi]);
        
        lpu = rot*lpu + [x(1);x(2);meas_model.height];
        rpu = rot*rpu + [x(1);x(2);meas_model.height];
        lpl = rot*lpl + [x(1);x(2);meas_model.height];
        rpl = rot*rpl + [x(1);x(2);meas_model.height];
        
%         rh = line([x(1);x(1) + r*cos(pan-(FoV/2))],[x(2);x(2) + r*sin(pan-(FoV/2))],...
%                         [0;-r*sin(psi)],'Color','red','LineStyle','--');
%         lh = line([x(1);x(1) + r*cos(pan+(FoV/2))],[x(2);x(2) + r*sin(pan+(FoV/2))],...
%                         [0;-r*sin(psi)],'Color','red','LineStyle','--');
        color = 'red';
        handles = [line([x(1);lpu(1)],[x(2);lpu(2)],[meas_model.height;lpu(3)],'Color',color,'LineStyle','-');
                   line([x(1);rpu(1)],[x(2);rpu(2)],[meas_model.height;rpu(3)],'Color',color,'LineStyle','-');
                   line([x(1);lpl(1)],[x(2);lpl(2)],[meas_model.height;lpl(3)],'Color',color,'LineStyle','-');
                   line([x(1);rpl(1)],[x(2);rpl(2)],[meas_model.height;rpl(3)],'Color',color,'LineStyle','-');
                   line([lpu(1);rpu(1)],[lpu(2);rpu(2)],[lpu(3);rpu(3)],'Color',color,'LineStyle','-');
                   line([rpu(1);rpl(1)],[rpu(2);rpl(2)],[rpu(3);rpl(3)],'Color',color,'LineStyle','-');
                   line([rpl(1);lpl(1)],[rpl(2);lpl(2)],[rpl(3);lpl(3)],'Color',color,'LineStyle','-');
                   line([lpl(1);lpu(1)],[lpl(2);lpu(2)],[lpl(3);lpu(3)],'Color',color,'LineStyle','-');
                   ];
                   
        
    end
    
    for i = 1:length(handles)
        handles(i).Annotation.LegendInformation.IconDisplayStyle = 'off';
    end
    
end

