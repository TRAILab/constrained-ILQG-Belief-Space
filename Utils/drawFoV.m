function [handles] = drawFoV(meas_model,x,handles, j)
%DRAWFOV Draws the FoV of the camera given robot pose
%   Input: 
%          meas_model - ObservationModelBase object
%          x   - robot pose [x,y,theta]
%          handles - handles for the FoV lines if we wish to replace
%                       existing ones
%          j - will draw every j'th states FoV, so fig is not cluttered 


%     if ~isempty(handles)
%         delete(handles);
%     end
    

%     figure(fig);
    hold on;
    K = size(x,2);
    new = 0;

    
    itc = round(linspace(1,length(x(1,:)),ceil(length(x(1,:))/j)));
    if isempty(handles)
        new = 1;
        if meas_model.obsDim == 2
            handles = gobjects(2,length(itc));
        else
            handles = gobjects(8,length(itc));
        end
    end
    handle_no = 1;
    for k = itc
        
        theta = x(3,k);
        r = 0.7;

        %draw max u line
        FoV = meas_model.FoV;
        if meas_model.obsDim == 2
            if new == 1

                handles(:,handle_no) = [line([x(1,k);x(1,k) + r*cos(theta-(FoV/2))],[x(2,k);x(2,k) + r*sin(theta-(FoV/2))],'Color','red','LineStyle','--');
                            line([x(1,k);x(1,k) + r*cos(theta+(FoV/2))],[x(2,k);x(2,k) + r*sin(theta+(FoV/2))],'Color','red','LineStyle','--');
                            ];
            else
                set(handles(1,handle_no),'XData',[x(1,k);x(1,k) + r*cos(theta-(FoV/2))],...
                                        'YData',[x(2,k);x(2,k) + r*sin(theta-(FoV/2))]);
                
                set(handles(2,handle_no),'XData',[x(1,k);x(1,k) + r*cos(theta+(FoV/2))],...
                                        'YData',[x(2,k);x(2,k) + r*sin(theta+(FoV/2))]);
            end
            
        else

            if length(x(:,1)) == 3
                phi = 0;
                psi = meas_model.psi;
            else
                phi = x(4,k);
                psi = x(5,k);
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

            lpu = rot*lpu + [x(1,k);x(2,k);meas_model.height];
            rpu = rot*rpu + [x(1,k);x(2,k);meas_model.height];
            lpl = rot*lpl + [x(1,k);x(2,k);meas_model.height];
            rpl = rot*rpl + [x(1,k);x(2,k);meas_model.height];

    %         rh = line([x(1,k);x(1,k) + r*cos(pan-(FoV/2))],[x(2,k);x(2,k) + r*sin(pan-(FoV/2))],...
    %                         [0;-r*sin(psi)],'Color','red','LineStyle','--');
    %         lh = line([x(1,k);x(1,k) + r*cos(pan+(FoV/2))],[x(2,k);x(2,k) + r*sin(pan+(FoV/2))],...
    %                         [0;-r*sin(psi)],'Color','red','LineStyle','--');
            if new == 1
                color = 'red';
                handles(:,handle_no) = [line([x(1,k);lpu(1)],[x(2,k);lpu(2)],[meas_model.height;lpu(3)],'Color',color,'LineStyle','-');
                           line([x(1,k);rpu(1)],[x(2,k);rpu(2)],[meas_model.height;rpu(3)],'Color',color,'LineStyle','-');
                           line([x(1,k);lpl(1)],[x(2,k);lpl(2)],[meas_model.height;lpl(3)],'Color',color,'LineStyle','-');
                           line([x(1,k);rpl(1)],[x(2,k);rpl(2)],[meas_model.height;rpl(3)],'Color',color,'LineStyle','-');
                           line([lpu(1);rpu(1)],[lpu(2);rpu(2)],[lpu(3);rpu(3)],'Color',color,'LineStyle','-');
                           line([rpu(1);rpl(1)],[rpu(2);rpl(2)],[rpu(3);rpl(3)],'Color',color,'LineStyle','-');
                           line([rpl(1);lpl(1)],[rpl(2);lpl(2)],[rpl(3);lpl(3)],'Color',color,'LineStyle','-');
                           line([lpl(1);lpu(1)],[lpl(2);lpu(2)],[lpl(3);lpu(3)],'Color',color,'LineStyle','-');
                           ];
            else
                set(handles(1,handle_no),'XData',[x(1,k);lpu(1)],'YData',[x(2,k);lpu(2)],'ZData',[meas_model.height;lpu(3)]);
                set(handles(2,handle_no),'XData',[x(1,k);rpu(1)],'YData',[x(2,k);rpu(2)],'ZData',[meas_model.height;rpu(3)]);
                set(handles(3,handle_no),'XData',[x(1,k);lpl(1)],'YData',[x(2,k);lpl(2)],'ZData',[meas_model.height;lpl(3)]);
                set(handles(4,handle_no),'XData',[x(1,k);rpl(1)],'YData',[x(2,k);rpl(2)],'ZData',[meas_model.height;rpl(3)]);
                set(handles(5,handle_no),'XData',[lpu(1);rpu(1)],'YData',[lpu(2);rpu(2)],'ZData',[lpu(3);rpu(3)]);
                set(handles(6,handle_no),'XData',[rpu(1);rpl(1)],'YData',[rpu(2);rpl(2)],'ZData',[rpu(3);rpl(3)]);
                set(handles(7,handle_no),'XData',[rpl(1);lpl(1)],'YData',[rpl(2);lpl(2)],'ZData',[rpl(3);lpl(3)]);
                set(handles(8,handle_no),'XData',[lpl(1);lpu(1)],'YData',[lpl(2);lpu(2)],'ZData',[lpl(3);lpu(3)]);
            end
            
            for i = 1:length(handles(:,handle_no))
                handles(i,handle_no).Annotation.LegendInformation.IconDisplayStyle = 'off';
            end
            
        end
        handle_no = handle_no + 1;

    end
    
end

