function Error = Flex_MB_BCS(InitGuess, ROBOT, THETA, THETA_DOT, THETA_DDOT, F_ext, c0, c1, c2)
% Flex-Rigid Multi-Body Open-Chain Manipulator Boundary Condition Solver to compute the
% difference between the constraint forces implied at the end of the last flexible body
% based on the assumption of constraint force at the beginning of the first flexible body
% compared to that implied by the external force applied at the EE.
%
% DETERMINE:    Error in the Constraint Force at end of Last Flexible Body
% GIVEN:        Manipulator Definition & Joint Position, Velocity, Acceleration &
%               Applied Loading & FDM Discretization Coeff.
%
%     - InitGuess:      Guess for the Strain at the Beginning of the First Elastic Element (guessed / iterated)
%     - ROBOT:          Definition of Manipulator (Collection of Bodies and Joints)
%     - THETA:          Actuated Joint Positions
%     - THETA_DOT:      Actuated Joint Velocities
%     - THETA_DDOT:     Actuated Joint Accelerations
%     - F_ext:          Applied Load at the EE expressed in EE BCF
%     - c0:             FDM Coeff - Current Time Step
%     - c1:             FDM Coeff - Previous Time Step
%     - c2:             FDM Coeff - PrePrevious Time Step
%     - RRC:            Robot Reference Configuration
%     - RAC:            Robot Actuated Configuration
%     - BCF:            Body Coordinate Frame
% 
% Written by BD Bhai

    [BC_Start,BC_End,NumBod] = MB_Flex_ID(ROBOT,THETA,THETA_DOT,THETA_DDOT);    %[]     Number of Bodies in the Manipulator
    if BC_End < BC_Start        %[]     Should only happen if there are no flexible members
        Error = zeros(6,1);     %[]     Terminate BCS as no need to iterate
        return
    end
    
    %% ALLOCATE MEMORY & INITIALIZE (N+2 for Bodies + Base + EE)
    g_ref = zeros(4,4,NumBod + 2);          %[SE(3) X N+2]  Transformation to i_th BCF from/in base BCF for RRC
    g_act_wrt_prev = zeros(4,4,NumBod + 2); %[SE(3) X N+2]  Transformation to i-1_th BCF from/in i_th BCF for RAC
    eta = zeros(6,NumBod + 2);              %[se(3) X N+2]  Body Velocity Twists for each BCF + Base + EE
    d_eta = zeros(6,NumBod + 2);            %[se(3) X N+2]  Body Acceleration Twists for each BCF + Base + EE Frame
    F = zeros(6,NumBod + 1);                %[se(3) X N+1]  Wrench for each BCF + Base
        
    % Set Initial Conditions
    g_ref(:,:,1) = eye(4);           %[SE(3)]    Base Frame Located @ Base Frame so Identity Transform
    g_act_wrt_prev(:,:,1) = eye(4);  %[SE(3)]    Base Frame Located @ Base Frame so Identity Transform
    eta(:,1) = zeros(6,1);           %[se(3)]    Base Frame Stationary so Twist zero
    d_eta(:,1) = zeros(6,1);         %[se(3)]    Base Frame Stationary so Twist Rate zero
    F_temp = zeros(6,1);
    
    %% RECURSIVE DEFINITION OF DYNAMICS USING EULER-POINCARE EOM
    for i = 2 : BC_End
        CoM2CoM = expm3(hat(ROBOT{2*(i-1)}.Parent.Transform - ROBOT{2*(i-1)}.Parent.CoM)) * ... %[SE(3)]    Transformation from Prev. Body CoM to Joint
                  expm3(hat(ROBOT{2*(i-1)}.Twist*ROBOT{2*(i-1)}.HomePos)) * ...                 %[SE(3)]    Transformation of Joint
                  expm3(hat(ROBOT{2*(i-1)}.Child.CoM));                                         %[SE(3)]    Transformation from Joint to Next Body CoM
        
        % Use Rigid-Body Kinematic Equations to find Velocities, Accelerations and Transformations
        % For Flexible-Body, I set CoM, Transform to 0 so this should just add joint actuation
        [g_ref(:,:,i),g_act_wrt_prev(:,:,i),eta(:,i),d_eta(:,i)] = Rigid_Kin(g_ref(:,:,i-1), CoM2CoM, ROBOT{2*(i-1)}, eta(:,i-1), d_eta(:,i-1));
        
        % This expresses F_temp from B coordinate frame in prev_body to C in curr_body
        F_temp = transpose(Ad(g_act_wrt_prev(:,:,i))) * F_temp;                                 %[N;Nm]
        
        if strcmp(ROBOT{2*i-1}.Type,'FLEXIBLE')
            if i == BC_Start                    %[]     Define the Iterated State for the BC
                F(:,i) = ROBOT{2*i-1}.Stiff * (InitGuess - ROBOT{2*i-1}.F_0);
            else
                F(:,i) = F_temp;
            end
            
            F_dist = zeros(6,ROBOT{2*i-1}.N);   %[N;Nm] No Distributed Applied Wrench Modeled Yet
            [g_ref(:,:,i),f_cur,eta_cur,d_eta(:,i)] = Flex_Dyn( g_ref(:,:,i), F_dist, F(:,i), ROBOT{2*i-1}, eta(:,i), c0, c1, c2);
            
            F_temp = ROBOT{2*i-1}.Stiff * (f_cur(:,end) - ROBOT{2*i-1}.F_0);     %[N;Nm]    Save wrench @ end as F_temp expressed in local BCF
            eta(:,i) = eta_cur(:,end);
            
        elseif i > BC_Start         %[]     Calculate Constraint Forces for all Rigid Bodies after first Elastic Body based on InitGuess
            F(:,i) =  F_temp;       %[N;Nm] Save Wrench Between i,i-1_th Body @ CoM Expressed in BCF
            
            F_temp = F(:,i) + transpose(adj(eta(:,i)))*ROBOT{2*i-1}.Mass*eta(:,i) ...
                - ROBOT{2*i-1}.Mass*d_eta(:,i);                                     %[N;Nm]     Save Wrench Between i,i-1_th Body @ Expressed in BCF [OK ?]
        end
    end
    
    %% ALGORITHM FOR LAST ELASTIC BODY TO END OF MANIPULATOR FOR BC LOADS 
    for i = BC_End + 1 : NumBod + 2
        CoM2CoM = expm3(hat(ROBOT{2*(i-1)}.Parent.Transform - ROBOT{2*(i-1)}.Parent.CoM)) * ... %[SE(3)]    Transformation from Prev. Body CoM to Joint
                  expm3(hat(ROBOT{2*(i-1)}.Twist*ROBOT{2*(i-1)}.HomePos)) * ...                 %[SE(3)]    Transformation of Joint (twist expressed in joint axis)
                  expm3(hat(ROBOT{2*(i-1)}.Child.CoM));                                         %[SE(3)]    Transformation from Joint to Next Body CoM
        
        % Use Rigid-Body Kinematic Equations to find Velocities, Accelerations and Transformations
        [g_ref(:,:,i),g_act_wrt_prev(:,:,i),eta(:,i),d_eta(:,i)] = Rigid_Kin(g_ref(:,:,i-1), CoM2CoM, ROBOT{2*(i-1)}, eta(:,i-1), d_eta(:,i-1));
    end
    
    F(:,end) = F_ext;                           %[N;Nm]     Assign Externally Applied Wrench @ EE (F_ext in Intertial so express in local)
    
    for i = flip(BC_End + 1 : NumBod + 1)
        F(:,i-1) = transpose(Ad(g_act_wrt_prev(:,:,i+1))) * F(:,i) + ...
            ROBOT{2*i-1}.Mass*d_eta(:,i) - transpose(adj(eta(:,i)))*ROBOT{2*i-1}.Mass*eta(:,i);     %[N;Nm]     Applied Wrench at i_th CoM  !!EQN FOR RIGID ONLY!!
    end                                         %[]         End of {SECTION III} Algorithm
    
    %[N,Nm]     Difference between Applied and Implied Wrench @ Wnd of Last Elastic Member  (ITERATE TO MAKE THIS ZERO)
    Error = F_temp - F(:,BC_End);   % Check if there's any coordinate change
    
end                     %[]     FUNCTION END