function [F, C, eta, ROBOT_New] = IDM_MultiBranch(ROBOT, F_ext, dt, InitGuessAll)
% Inverse Dynamic Model for Multi-Branch Open-Chain Manipulator (!! RIGID & FLEXIBLE !!)
% Written by BD Bhai
% 
% DETERMINE:    Constraint, Actutation Forces & Body Velocities
% GIVEN:        Manipulator Definition & Joint Position, Velocity, Acceleration
% 
%     - ROBOT:          Definition of Manipulator (Collection of bodies and joints)
%     - THETA:          Actuated Joint Positions
%     - THETA_DOT:      Actuated Joint Velocities
%     - THETA_DDOT:     Actuated Joint Accelerations
%     - F_ext:          Applied Load at the EE expressed in EE BCF
%     - InitGuess:      Initial Guess for Strain of 1st Flexible Body

%     [BC_Start,~,NumBod] = MB_Flex_ID(ROBOT,THETA,THETA_DOT,THETA_DDOT);     %[]     Number of Bodies & ID Flexible
    NumBod = 7;     %[]     Num Bodies in each Branch
    
    ROBOT_New = ROBOT;                       %[]     Initialize Structure to Save Updated History for BDF-2
    
    %% Initialize Memory for Parameters Defined Recursively (N+3 for Bodies + Base + 2xEE) 
    % ROBOT{2*i-2} = i_th Joint  (including JointEE as (N+1)_th Joint)
    % ROBOT{2*i-1} = i_th Body   (including Base as 1st Body and EE as (N+2)_th Body)
    g_ref = zeros(4,4,NumBod + 3);           %[SE(3) X N+3]  Transformation to i_th C-BCF from/in base BCF for RRC
    g_act_wrt_prev = zeros(4,4,NumBod + 3);  %[SE(3) X N+3]  Transformation to i-1_th C-BCF from/in i_th BCF for RAC
    eta = zeros(6,NumBod + 3);               %[se(3) X N+3]  Twists for each BCF + Base + EE in BCF
    d_eta = zeros(6,NumBod + 3);             %[se(3) X N+3]  Twist Rate for each BCF + Base + EE Frame in BCF
    F = zeros(6,NumBod + 2);                 %[se(3) X N+2]  Wrench for each Joint + EE in BCF
    C = zeros(1,NumBod);                     %[se(3) X N]    Actuated Control for each Joint in BCF
    
    %% Implement BDF-2 to Approximate Time-Derivatives & Solve BC (if relevant) 
    c0 = 1.5/dt; c1 = -2/dt; c2 = .5/dt;
    options = optimset('Display','OFF','TolFun',1e-9);
%     [InitGuess,~,~,OUT] = fsolve(@(InitGuess)Flex_MB_BCS(InitGuess, ROBOT, THETA, THETA_DOT, THETA_DDOT, F_ext, dt, c0, c1, c2),InitGuess,options);
%     OUT.iterations
    
    %% {SECTION I & II & III} - ALGORITHM FOR ALL RIGID BODIES PRIOR TO FIRST FLEXIBLE BODY 
    g_ref(:,:,1) = eye(4);          %[SE(3)]    Base Frame Located @ Base Frame so Identity Transform
    g_act_wrt_prev(:,:,1) = eye(4); %[SE(3)]    Base Frame Located @ Base Frame so Identity Transform
    eta(:,1) = zeros(6,1);          %[se(3)]    Base Frame Stationary so Twist Zero
    d_eta(:,1) = zeros(6,1);        %[se(3)]    Base Frame Stationary so Twist Rate Zero
    
    F_temp = zeros(6,1);
%     THETA = [THETA;0];              %[]     Add zero Joint Angle to Joint_EE (UPDATE FOR 2 EE)
    
    %% Iterate Through 1st 3 Rigid Bodies
    for i = 2:4                     %[]     Iterate through all rigid bodies
        CoM2CoM = expm3(hat(ROBOT{2*(i-1)}.Parent.Transform - ROBOT{2*(i-1)}.Parent.CoM)) * ... %[SE(3)]    Transformation from Prev. Body CoM to Joint
                  expm3(hat(ROBOT{2*(i-1)}.Twist*ROBOT{2*(i-1)}.HomePos)) * ...                 %[SE(3)]    Transformation of Joint (twist expressed in joint axis)
                  expm3(hat(ROBOT{2*(i-1)}.Child.CoM));                                         %[SE(3)]    Transformation from Joint to Next Body CoM
        
        % Use Rigid-Body Kinematic Equations to find Velocities, Accelerations and Transformations
        % For Flexible-Body, I set CoM, Transform to Identity so just adds joint actuation
        [g_ref(:,:,i),g_act_wrt_prev(:,:,i),eta(:,i),d_eta(:,i)] = Rigid_Kin(g_ref(:,:,i-1), CoM2CoM, ROBOT{2*(i-1)}, eta(:,i-1), d_eta(:,i-1));
        
    end
    
    %% Iterate Through Each Flexible Branch
    for i = 1:2
        SUB_SYS = ROBOT([ 1:7, (6*i + (2:7)) ]);
        InitGuess = InitGuessAll(:,i);
        [InitGuess,~,~,~] = fsolve(@(InitGuess)Flex_MultiBranch_BCS(InitGuess, eta(:,4), SUB_SYS(8:13), F_ext(:,i), c0, c1, c2),InitGuess,options);
        for index = 5:7
            CoM2CoM = expm3(hat(SUB_SYS{2*(index-1)}.Parent.Transform - SUB_SYS{2*(index-1)}.Parent.CoM)) * ... %[SE(3)]    Transformation from Prev. Body CoM to Joint
                  expm3(hat(SUB_SYS{2*(index-1)}.Twist*SUB_SYS{2*(index-1)}.HomePos)) * ...                     %[SE(3)]    Transformation of Joint (twist expressed in joint axis)
                  expm3(hat(SUB_SYS{2*(index-1)}.Child.CoM));                                                   %[SE(3)]    Transformation from Joint to Next Body CoM
            
%             [g_ref(:,:,index + 3*(i-1)),g_act_wrt_prev(:,:,index + 3*(i-1)),eta(:,index + 3*(i-1)),d_eta(:,index + 3*(i-1))] = ...
%                 Rigid_Kin(g_ref(:,:,index + 3*(i-1) - 1), CoM2CoM, SUB_SYS{2*(index-1)}, eta(:,index + 3*(i-1) - 1), d_eta(:,index + 3*(i-1) - 1));
            
            if index == 5   %[]         Define the Iterated State for the BC
                [g_ref(:,:,5 + 3*(i-1)),g_act_wrt_prev(:,:,index + 3*(i-1)),eta(:,index + 3*(i-1)),d_eta(:,index + 3*(i-1))] = ...
                Rigid_Kin(g_ref(:,:,5 - 1), CoM2CoM, SUB_SYS{2*(index-1)}, eta(:,index + 3*(i-1) - 1), d_eta(:,index + 3*(i-1) - 1));
                
                F(:,index + 3*(i-1)) = SUB_SYS{2*index-1}.Stiff * (InitGuess - SUB_SYS{2*index-1}.F_0);
            else
                [g_ref(:,:,index + 3*(i-1)),g_act_wrt_prev(:,:,index + 3*(i-1)),eta(:,index + 3*(i-1)),d_eta(:,index + 3*(i-1))] = ...
                Rigid_Kin(g_ref(:,:,index + 3*(i-1) - 1), CoM2CoM, SUB_SYS{2*(index-1)}, eta(:,index + 3*(i-1) - 1), d_eta(:,index + 3*(i-1) - 1));
                
                F(:,index + 3*(i-1)) = F_temp;
            end
            C(:,index + 3*(i-1)-1) = transpose(F(:,index + 3*(i-1))) * SUB_SYS{2*(index-1)}.Twist;

            if strcmp(SUB_SYS{2*index-1}.Type,'FLEXIBLE')
                F_dist = zeros(6,SUB_SYS{2*index-1}.N);                                             %[N,Nm]     No Distributed Applied Wrench Modeled Yet
                
                if index == 5
                    eta_prev = eta(:,4);
                else
                    eta_prev = eta(:,index + 3*(i-1));
                end
                
                [g_ref(:,:,index + 3*(i-1)),f_cur,eta_cur,d_eta(:,index + 3*(i-1))] = ...
                    Flex_Dyn( g_ref(:,:,index + 3*(i-1)), F_dist, F(:,index + 3*(i-1)), SUB_SYS{2*index-1}, eta_prev, c0, c1, c2);

                F_temp = SUB_SYS{2*index-1}.Stiff * (f_cur(:,end) - SUB_SYS{2*index-1}.F_0);        %[N;Nm]     Save wrench @ end of continuum as F_temp expressed in local BCF
                eta(:,index + 3*(i-1)) = eta_cur(:,end);

                %% Update History Terms (after all computations complete)
                ROBOT_New{2*index-1 + 6*(i-1)}.eta_pprev = SUB_SYS{2*index-1}.eta_prev;
                ROBOT_New{2*index-1 + 6*(i-1)}.eta_prev = eta_cur;

                ROBOT_New{2*index-1 + 6*(i-1)}.f_pprev = SUB_SYS{2*index-1}.f_prev;
                ROBOT_New{2*index-1 + 6*(i-1)}.f_prev = f_cur;
            end
        end
    end
        
    %% Backward Iterations - Determine Constraint Forces (ALL RIGID BODIES) (FIX INDEXING)
    % DEBUG (NEED TO FIX CONSTRIANT FORCES FOR FLEXIBLE BODIES)
%     F(:,end) = F_ext;
    
%   For Body_3 with 2 subsequent bodies
    F(:,4) = transpose(Ad(g_act_wrt_prev(:,:,5+1))) * F(:,5) + transpose(Ad(g_act_wrt_prev(:,:,8+1))) * F(:,8) + ...
            ROBOT{2*4-1}.Mass*d_eta(:,4) - transpose(adj(eta(:,4)))*ROBOT{2*4-1}.Mass*eta(:,4);
    C(:,4-1) = transpose(F(:,4)) * Ad(expm3(hat(-ROBOT{2*4-1}.CoM)))*ROBOT{2*(4-1)}.Twist;      %[] Transform Joint twist to Link CoM BCF then Project Constraints
    
    for i = [3,2]
        F(:,i) = transpose(Ad(g_act_wrt_prev(:,:,i+1))) * F(:,i+1) + ...
            ROBOT{2*i-1}.Mass*d_eta(:,i) - transpose(adj(eta(:,i)))*ROBOT{2*i-1}.Mass*eta(:,i);
        C(:,i-1) = transpose(F(:,i-1)) * Ad(expm3(hat(-ROBOT{2*i-1}.CoM)))*ROBOT{2*(i-1)}.Twist;      %[] Transform Joint twist to Link CoM BCF then Project Constraints
    end
    C = transpose(C);
    C(6) = [];    %[]     Remove EE_1
    C(end) = [];  %[]     Remove EE_2
    
end