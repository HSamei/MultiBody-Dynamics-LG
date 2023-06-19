function [F, C, eta, ROBOT_New] = IDM_MB_RE(ROBOT, THETA, THETA_DOT, THETA_DDOT, F_ext, dt, InitGuess)
% Inverse Dynamic Model for Rigid-Flexible Multi-Body Open-Chain Manipulator
% 
% DETERMINE:    Constraint, Actutation Forces & Body Velocities
% GIVEN:        Manipulator Definition & Joint Position, Velocity, Acceleration
% 
%     - InitGuess:      Initial Guess for Strain at the Beginning of 1st Flexible Body
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

    [BC_Start,~,NumBod] = MB_Flex_ID(ROBOT,THETA,THETA_DOT,THETA_DDOT);     %[]     Number of Bodies & ID Flexible
    ROBOT_New = ROBOT;                                                      %[]     Initialize Structure to Save Updated History for BDF-2
    
    %% ALLOCATE MEMORY (N+2 for Bodies + Base + EE)
    % ROBOT{2*i-2} = i_th Joint  (including JointEE as (N+1)_th Joint)
    % ROBOT{2*i-1} = i_th Body   (including Base as 1st Body and EE as (N+2)th Body)
    g_ref = zeros(4,4,NumBod + 2);           %[SE(3) X N+2]  Transformation to i_th C-BCF from/in base BCF for RRC
    g_act_wrt_prev = zeros(4,4,NumBod + 2);  %[SE(3) X N+2]  Transformation to i-1_th C-BCF from/in i_th BCF for RAC
    eta = zeros(6,NumBod + 2);               %[se(3) X N+2]  Twists for each BCF + Base + EE in BCF
    d_eta = zeros(6,NumBod + 2);             %[se(3) X N+2]  Twist Rate for each BCF + Base + EE Frame in BCF
    F = zeros(6,NumBod + 1);                 %[se(3) X N+1]  Wrench for each Joint + EE in BCF
    C = zeros(1,NumBod);                     %[se(3) X N]    Actuated Control for each Joint in BCF
    
    %% IMPLEMENT FDM & BCS
    c0 = 1.5/dt; c1 = -2/dt; c2 = .5/dt;                %[]     FDM Coefficients for BDF-2
    options = optimset('Display','OFF','TolFun',1e-9);
    [InitGuess,~,~,~] = fsolve(@(InitGuess)Flex_MB_BCS(InitGuess, ROBOT, THETA, THETA_DOT, ...
        THETA_DDOT, F_ext, c0, c1, c2),InitGuess,options);
    
    %% RECURSIVE DEFINITION OF DYNAMICS USING EULER-POINCARE EOM
    g_ref(:,:,1) = eye(4);           %[SE(3)]    Base Frame Located @ Base Frame so Identity Transform
    g_act_wrt_prev(:,:,1) = eye(4);  %[SE(3)]    Base Frame Located @ Base Frame so Identity Transform
    eta(:,1) = zeros(6,1);           %[se(3)]    Base Frame Stationary so Twist Zero
    d_eta(:,1) = zeros(6,1);         %[se(3)]    Base Frame Stationary so Twist Rate Zero
    
    F_temp = zeros(6,1);
    THETA = [THETA;0];               %[]     Add zero Joint Angle to Joint_EE
    
    for i = 2 : NumBod + 2           %[]     Iterate through all Bodies
        CoM2CoM = expm3(hat(ROBOT{2*(i-1)}.Parent.Transform - ROBOT{2*(i-1)}.Parent.CoM)) * ... %[SE(3)]    Transformation from Prev. Body CoM to Joint
                  expm3(hat(ROBOT{2*(i-1)}.Twist*ROBOT{2*(i-1)}.HomePos)) * ...                 %[SE(3)]    Transformation of Joint (twist expressed in joint axis)
                  expm3(hat(ROBOT{2*(i-1)}.Child.CoM));                                         %[SE(3)]    Transformation from Joint to Next Body CoM
        
        % Use Rigid-Body Kinematic Equations to find Velocities, Accelerations and Transformations
        % For Flexible-Body, I set CoM, Transform to Identity so just adds joint actuation
        [g_ref(:,:,i),g_act_wrt_prev(:,:,i),eta(:,i),d_eta(:,i)] = Rigid_Kin(g_ref(:,:,i-1), CoM2CoM, ROBOT{2*(i-1)}, eta(:,i-1), d_eta(:,i-1));
        
        % Use if F_temp is expressed at CoM
        F_temp = transpose(Ad(g_act_wrt_prev(:,:,i))) * F_temp;                                 %[N;Nm]     Save wrench @ CoM as F(:,i) expressed in local BCF [OK?]
        
        if strcmp(ROBOT{2*i-1}.Type,'FLEXIBLE')
            if i == BC_Start       %[]         Define the Iterated State for the BC
                F(:,i) = ROBOT{2*i-1}.Stiff * (InitGuess - ROBOT{2*i-1}.F_0);
            else
                F(:,i) = F_temp;
            end
            C(:,i-1) = transpose(F(:,i)) * ROBOT{2*i-2}.Twist;
            
            F_dist = zeros(6,ROBOT{2*i-1}.N);                                                   %[N,Nm]     No Distributed Applied Wrench Modeled Yet
            [g_ref(:,:,i),f_cur,eta_cur,d_eta(:,i)] = Flex_Dyn( g_ref(:,:,i), F_dist, F(:,i), ROBOT{2*i-1}, eta(:,i), c0, c1, c2);
            
            F_temp = ROBOT{2*i-1}.Stiff * (f_cur(:,end) - ROBOT{2*i-1}.F_0);                    %[N;Nm]     Save wrench @ end of continuum as F_temp expressed in local BCF
            eta(:,i) = eta_cur(:,end);
            
            % Update History Terms (After all computations complete)
            ROBOT_New{2*i-1}.eta_pprev = ROBOT{2*i-1}.eta_prev;
            ROBOT_New{2*i-1}.eta_prev = eta_cur;
            
            ROBOT_New{2*i-1}.f_pprev = ROBOT{2*i-1}.f_prev;
            ROBOT_New{2*i-1}.f_prev = f_cur;
            
        elseif i > BC_Start         %[]     Calculate Constraint Forces for all Rigid Bodies after first Elastic Body based on InitGuess
            F(:,i) = F_temp;        %[N;Nm] Save wrench @ CoM as F(:,i) expressed in local BCF
            if i < NumBod + 2       %[]     Don't save it for the EE stuff
                C(:,i-1) = transpose(F(:,i)) * ROBOT{2*i-2}.Twist;
            end
            
            F_temp = (F(:,i) + transpose(adj(eta(:,i)))*ROBOT{2*i-1}.Mass*eta(:,i) ...
                - ROBOT{2*i-1}.Mass*d_eta(:,i));                                                %[N;Nm]     Save applied wrench @ end of Link as F_temp expressed in CoM BCF
        end
    end                             %[]     end of {SECTION II} Algorithm
    
    % Backward Iterations - Determine Constraint Forces (all remaining bodies are rigid) 
    F(:,end) = F_ext;
    
    for i = flip(2 : BC_Start)
        F(:,i-1) = transpose(Ad(g_act_wrt_prev(:,:,i+1))) * F(:,i) + ...
            ROBOT{2*i-1}.Mass*d_eta(:,i) - transpose(adj(eta(:,i)))*ROBOT{2*i-1}.Mass*eta(:,i);
        C(:,i-1) = transpose(F(:,i-1)) * Ad(expm3(hat(-ROBOT{2*i-1}.CoM)))*ROBOT{2*(i-1)}.Twist;      %[] Transform Joint twist to Link CoM BCF then Project Constraints
    end
    C = transpose(C);
end