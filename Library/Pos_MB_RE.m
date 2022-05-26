function [POS,POS_R,POS_F] = Pos_MB_RE(ROBOT,THETA,NumStep)
% Post-Processing Visualization: Integrate Elements for Positon to Plot/Animate ROBOT Configuration
% #ok<*VUNUS>
% 
% DETERMINE:    Position Nodes for Visualization of ROBOT
% GIVEN:        ROBOT Definition, Joint Angles, Velocities, Accelerations
% 
%     - POS:            Position Nodes for All Bodies in System
%     - POS_R:          Position Nodes for Rigid Bodies in System
%     - POS_F:          Position Nodes for Flexible Bodies in System
%     - ROBOT:          Definition of Manipulator (Collection of Bodies and Joints)
%     - THETA:          Actuated Joint Positions
%     - NumStep:        Factor to scale ds by for visual plotting
% 
% Written by BD Bhai

    %% Compute Robot Positions
    % assumes f_prev records the strains for each flexible body in current time-step (run dynamics, update value, pass to plotter)
    POS = zeros(3,1);       %[R3]   Will be updated to include each point that is plotted
    POS_R = zeros(3,1);     %[R3]   Will be updated to include each point that is plotted
    POS_F = zeros(3,1);     %[R3]   Will be updated to include each point that is plotted
    g = eye(4);             %[SE3]  Transformation for the base of the manipulator
    iii = 2;                %[]     Index to increment into POS, will be number of points to be plotted in the end
    i_R = 1;                %[]     Index to increment into POS_R, will be number of points to be plotted in the end
    i_F = 1;                %[]     Index to increment into POS_F, will be number of points to be plotted in the end
    
    for i = 1:(length(ROBOT) - 3) / 2                                               %[Body]     Correspond to the i_th Body in the Robot
        if strcmpi(ROBOT{2*i+1}.Type,'RIGID')
            g = g * expm3(hat(ROBOT{2*i}.Twist * (ROBOT{2*i}.HomePos + THETA(i)))); %[SE(3)]    Transformation to After Joint_{i,i-}
            POS(:,iii) = g(1:3,4); iii = iii + 1;                                   %[R3]       Save Position at Beginning of Link, index to next
            POS_R(:,i_R) = g(1:3,4); i_R = i_R + 1;                                 %[R3]       Save Position at Beginning of Link, index to next
            g = g * expm3(hat(ROBOT{2*i}.Child.Transform));                         %[SE(3)]    Transformation to After Body_{i}
            POS(:,iii) = g(1:3,4); iii = iii + 1;                                   %[R3]       Save Position at End of Link, index to next
            POS_R(:,i_R) = g(1:3,4); i_R = i_R + 1;                                 %[R3]       Save Position at Beginning of Link, index to next
        elseif strcmpi(ROBOT{2*i+1}.Type,'FLEXIBLE')
            g = g * expm3(hat(ROBOT{2*i}.Twist * (ROBOT{2*i}.HomePos + THETA(i)))); %[SE(3)]    Transformation to After Joint_{i,i-}
            ds = ROBOT{2*i+1}.L / ROBOT{2*i+1}.N;                                   %[m]        Size of Differential Element (used to compute f_prev)
            
            for ii = 1:(ROBOT{2*i+1}.N * NumStep)
                index = ceil(ii/NumStep);                                           %[]         Determine the Local Strain Value at Step
                g = g * expm3(hat(ROBOT{2*i+1}.f_prev(:,index)) * ds/NumStep);      %[SE(3)]    Transformation to s = s + ds/NumStep
                POS(:,iii) = g(1:3,4); iii = iii + 1;                               %[R3]       Save Position at s = s + ds/NumStep, index to next
                POS_F(:,i_F) = g(1:3,4); i_F = i_F + 1;                             %[R3]       Save Position at s = s + ds/NumStep, index to next
            end
        end
    end
end