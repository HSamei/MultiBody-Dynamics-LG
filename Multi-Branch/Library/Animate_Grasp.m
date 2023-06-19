function [imind,cm,g_EE] = Animate_Grasp(Robot,Theta,NumStep,NumBod,PlotObj,Fig)
% Post-Processing Visualization: Plot Object for Each Body
% #ok<*VUNUS>
% 
% DETERMINE:    Position Nodes for Visualization of ROBOT
% GIVEN:        ROBOT Definition, Joint Angles, Velocities, Accelerations
% 
% Written by BD Bhai
    
    [~,POS_R,POS_F,g_EE] = Pos_MB_RE(Robot,Theta,NumStep);
    Start_F = 1;        % Record the index in POS_F
    Count_R = 1;        % Record the index in POS_R
    
    % Plot the Bodies
    for ii = 1:NumBod-2
        clearpoints(PlotObj{ii})
        if strcmp(Robot{2*ii+1}.Type,'FLEXIBLE')        % Linestyle for Flexible Bodies
            N_Body = Robot{2*ii+1}.N * NumStep;         % Determine number of data points in body
            End_F = Start_F + N_Body - 1;               % Identify end index of the body
            addpoints(PlotObj{ii},POS_F(1,Start_F:End_F),POS_F(2,Start_F:End_F),POS_F(3,Start_F:End_F));
            Start_F = End_F + 1;
        elseif strcmp(Robot{2*ii+1}.Type,'RIGID')        % Linestyle for Rigid Bodies
            addpoints(PlotObj{ii},POS_R(1,Count_R:Count_R+1),POS_R(2,Count_R:Count_R+1),POS_R(3,Count_R:Count_R+1));
            Count_R = Count_R + 2;
        end
    end
    for ii = NumBod - [1,0]
        clearpoints(PlotObj{ii})
        if strcmp(Robot{2*ii+3}.Type,'FLEXIBLE')        % Linestyle for Flexible Bodies
            N_Body = Robot{2*ii+3}.N * NumStep;         % Determine number of data points in body
            End_F = Start_F + N_Body - 1;               % Identify end index of the body
            addpoints(PlotObj{ii},POS_F(1,Start_F:End_F),POS_F(2,Start_F:End_F),POS_F(3,Start_F:End_F));
            Start_F = End_F + 1;
        elseif strcmp(Robot{2*ii+3}.Type,'RIGID')        % Linestyle for Rigid Bodies
            addpoints(PlotObj{ii},POS_R(1,Count_R:Count_R+1),POS_R(2,Count_R:Count_R+1),POS_R(3,Count_R:Count_R+1));
            Count_R = Count_R + 2;
        end
    end
    drawnow, pause(.05)
    
    % Save the frame for output animation file
    im = frame2im(getframe(Fig));
    [imind,cm] = rgb2ind(im,256);
end