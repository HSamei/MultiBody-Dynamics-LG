function Error = Flex_MultiBranch_BCS(InitGuess, eta_0, ROBOT, F_ext, c0, c1, c2)
% Boundary Condition Solver for Multi-Body Open-Chain Manipulator (!! RIGID & FLEXIBLE !!)
% Written by BD Bhai
%
% DETERMINE:    Constraint, Actutation Forces & Body Velocities
% GIVEN:        Manipulator Definition & Joint Position, Velocity, Acceleration & Applied Loading
%
%     - InitGuess:      Guess for the Strain at the Beginning of the First Elastic Element
%     - ROBOT:          Definition of Manipulator (Collection of Bodies and Joints)
%     - THETA:          Actuated Joint Positions
%     - THETA_DOT:      Actuated Joint Velocities
%     - THETA_DDOT:     Actuated Joint Accelerations
%     - F_ext:          Applied Load at the EE expressed in EE BCF
    
    %% Initialize Memory for Parameters Defined Recursively (N+2 for Bodies + Base + EE)
%     THETA(end + 1) = 0;         %[rad]          Add 0 angle for 'joint' to EE
    NumBod = 3;         
    eta = zeros(6,NumBod + 1);  %[se(3) X N]    Twists for each BCF + EE in BCF
%     F = zeros(6,NumBod);        %[se(3) X N+1]  Wrench for each BCF + Base in BCF
    
    % Set Initial Conditions
    eta(:,1) = eta_0;           %[se(3)]        Base Frame Stationary so Twist zero
    d_eta(:,1) = zeros(6,1);    %[se(3)]        Base Frame Stationary so Twist Rate zero
    F_temp = zeros(6,1);
    
    %% ASSUME BASE STRESS/STRAIN AT BEGINNING OF FIRST FLEXIBLE BODY AND SIMULATE EQUATIONS UP TO LAST ELASTIC BODY (HERE BEGINS THE TOMFOOLERY) {SECTION II}
    for i = 1 : NumBod
        CoM2CoM = expm3(hat(ROBOT{2*i-1}.Parent.Transform - ROBOT{2*i-1}.Parent.CoM)) * ... %[SE(3)]    Transformation from Prev. Body CoM to Joint
                  expm3(hat(ROBOT{2*i-1}.Twist*ROBOT{2*i-1}.HomePos)) * ...                 %[SE(3)]    Transformation of Joint (twist expressed in joint axis)
                  expm3(hat(ROBOT{2*i-1}.Child.CoM));                                       %[SE(3)]    Transformation from Joint to Next Body CoM
        
        % Use Rigid-Body Kinematic Equations to find Velocities, Accelerations and Transformations
        % For Flexible-Body, I set CoM, Transform to 0 so this should just add joint actuation
        [~,g_temp,eta(:,i+1),~] = Rigid_Kin(eye(4), CoM2CoM, ROBOT{2*i-1}, eta(:,i), zeros(6,1));
        
        % This expresses F_temp from B coordinate frame in prev_body to C in curr_body
        F_temp = transpose(Ad(g_temp)) * F_temp;    %[N;Nm]
        
        if strcmp(ROBOT{2*i}.Type,'FLEXIBLE')
            if i == 1                        %[]         Define the Iterated State for the BC
                F = ROBOT{2*i}.Stiff * (InitGuess - ROBOT{2*i}.F_0);
            else
                F = F_temp;
            end
            
            F_dist = zeros(6,ROBOT{2*i}.N);       %[N,Nm]     No Distributed Applied Wrench Modeled Yet
            [~,f_cur,eta_cur,d_eta(:,i)] = Flex_Dyn( eye(4), F_dist, F, ROBOT{2*i}, eta(:,i+1), c0, c1, c2);
            
            F_temp = ROBOT{2*i}.Stiff * (f_cur(:,end) - ROBOT{2*i}.F_0);     %[N;Nm]     Save wrench @ end of continuum as F_temp expressed in local BCF
            eta(:,i+1) = eta_cur(:,end);
            
        end
    end                             %[]     end of {SECTION II} Algorithm
    
    %[N,Nm]     Difference between Applied and Implied Wrench @ Wnd of Last Elastic Member  (ITERATE TO MAKE THIS ZERO)
    Error = F_temp - F_ext;   % Check if there's any coordinate change
    
end                     %[]     FUNCTION END