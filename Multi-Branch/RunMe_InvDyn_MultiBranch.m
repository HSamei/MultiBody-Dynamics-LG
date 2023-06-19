%% %%%%%%%%%%%%% Rigid Open-Chain Multi-Body Inverse Dynamic Calculator %%%%%%%%%%%%% %%
% Written by BD Bhai
%#ok<*CCAT>
% CAN DEFINE TRANSFORMATIONS INTO JOINT OR BODY. CHOOSE JOINTS SO CAN EASILY
% IDENTIFY THE BODY FRAME @ COM AND JOINT FRAME IS NOT AS USEFUL TO TRACK.
% 
% RRC - Robot Reference Configuration
% RAC - Robot Actuated Configuration
% BCF - Body Coordinate Frame

%% Initialize System        % STRETCH    (Read from '.URDF' file)
% RefModel = importrobot('iiwa14.urdf');
% clear
addpath(strcat(pwd,'/Library'))
% tic

%% SIMULATION PARAMETERS
%% Param for defMultiBranch()
% Initial Conditions
Theta       = [0;-pi/2;0;0;0;0;0];      %[]     Actuated Joint Position
Theta_dot   = [0;0;0;0;0;0;0];          %[]     Actuated Joint Velocity

% Joint Trajectory
Theta_ddot(:,1)  = [0;25;0;0;0;0;0];        %[]     Actuated Joint Acceleration
Theta_ddot(:,2)  = -Theta_ddot(:,1);        %[]     Actuated Joint Acceleration
Theta_ddot(:,3)  = [0;0;0;-4;-7.5;4;7.5]*1.05;   %[]     Actuated Joint Acceleration
Theta_ddot(:,4)  = -Theta_ddot(:,3);        %[]     Actuated Joint Acceleration
Theta_ddot(:,5)  = [0;0;0;0;0;0;0];         %[]     Actuated Joint Acceleration
Theta_ddot(:,6)  = [0;0;0;-4;-7.5;4;7.5]/10;%[]     Actuated Joint Acceleration
Theta_ddot(:,7)  = -Theta_ddot(:,6);        %[]     Actuated Joint Acceleration
Theta_ddot(:,8)  = [0;0;0;0;0;0;0];         %[]     Actuated Joint Acceleration

% FDM and System Definitions
dt          = .025 ;         %[s]    Time Step Size
TimeStep    = [10,10,10,10,30,30,30,30] ;           %[]     Number of Time Steps Evaluated
VisScale    = .5;          %[]     Scale to Axes Dimensions if System Out of Plot View
NumStep     = 2;            %[]     Factor to shrink ds for Flexible Bodies
RestTime    = 5;            %[]     Hold Joint Rates at 0 for RestTime*dt

F_ext       = zeros(6,2);           %[N;Nm] Applied Wrench at the EE
F_0         = [0;0;1;0;0;0];        %[]     Free Strain in first Flexible Body
F_0         = [F_0,F_0];            %[]     For the 2 Flexible Body/Branches
filename    = 'TestPaper_Multi.gif';    %[]     Filename for Output Video
SNAP        = [1,25,50,0];          %[]     TimeStep to Plot Snapshot (ascending order and padd with 0)
% Robot       = defMultiBranch(Theta,Theta_dot,Theta_ddot);
Robot       = defMultiBranch_Paper(Theta,Theta_dot,Theta_ddot);

%% INITIALIZATION FOR RESPONSE LOOP
% [BC_Start,BC_End,NumBod] = MB_Flex_ID(Robot,Theta(1:5),Theta_dot(1:5),Theta_ddot(1:5));     %[]     Number of Bodies & ID Flexible
BC_Start = 5;
BC_End = 6;
NumBod = 7;
t = dt:dt:dt*(sum(TimeStep));             %[s]    Time Vector
C = zeros(NumBod,sum(TimeStep));          %[N,Nm] Allocate Memory to save the Control Torques
T_H = zeros(NumBod,sum(TimeStep));        %[N,Nm] Allocate Memory to save the Joint Angles
F_H = zeros(3,sum(TimeStep));             %[N,Nm] Allocate Memory to save the Tip Force
EE_POS = zeros(3,2,sum(TimeStep));          %[N,Nm] Allocate Memory to save the EE Position
Robo_SNAP{length(SNAP)} = 0;              %[]     Allocate Memory to save the Robot States for SNAP
PrevGuess = 1;                            %[]     Use Previous Guess (1) or BDF-2 Mod Guess (0)
Animate = 1;                              %[]     Generate Animation Video and File (1)
snap_count = 1;                           %[]     Counter for Saving the Snapshots Data
data_index = 0;                           %[]     Counter for Saving the Data

% Initialization of Objects for Animation
if Animate 
    Fig = figure(21); Axes = gca;
    
    % Define Target Object (cylinder)
    Target_R = .2;
    Target_Offset = .7;
    Target_Height = .25;
    [Target_X,Target_Y,Target_Z] = cylinder(Target_R);
    Target = surf(Axes, Target_X, Target_Y - Target_Offset, Target_Z*Target_Height);
    
    view(3), axis equal, axis(VisScale*[-1 1 -2 .5 -.01 2]), grid on;
    
    % Define Cell Array to Store Handle for each Body
    PlotObj{NumBod} = 0;                        %[]     Initialize Cell Array Size
    for i = 1:NumBod-2                          %[]     Make Handle for each Body
        if strcmp(Robot{2*i+1}.Type,'FLEXIBLE')         % Linestyle for Flexible Bodies
            PlotObj{i} = animatedline(Axes,'Color','r','LineStyle','none','Marker','.','MarkerSize',10);
        elseif strcmp(Robot{2*i+1}.Type,'RIGID')        % Linestyle for Rigid Bodies
            PlotObj{i} = animatedline(Axes,'Color','k','LineStyle','-','LineWidth',3,'Marker','.');
        end
    end
    for i = NumBod - [1,0]                      %[]     Make Handle for each Body
        if strcmp(Robot{2*i+3}.Type,'FLEXIBLE')         % Linestyle for Flexible Bodies
            PlotObj{i} = animatedline(Axes,'Color','g','LineStyle','none','Marker','.','MarkerSize',10);
        elseif strcmp(Robot{2*i+3}.Type,'RIGID')        % Linestyle for Rigid Bodies
            PlotObj{i} = animatedline(Axes,'Color','g','LineStyle','-','LineWidth',3,'Marker','.');
        end
    end
end

%% RUN LOOP FOR TIME RESPONSE
for ii = 1:length(TimeStep)
    % Update Constant Acceleration Trajectory
    Robot = Update_Joints(Robot,Theta,Theta_dot,Theta_ddot(:,ii));
    
    % Compute for defined TimeSteps
    for i = 1:TimeStep(ii) 
        data_index = data_index + 1;
        [F,C(:,data_index),eta,Robot] = IDM_MultiBranch(Robot,F_ext,dt,F_0);             %[] Compute system states at next timestep 
        [T_H(:,data_index),Theta,Theta_dot] = Step_Joints(Theta,Theta_dot,Theta_ddot(:,ii),dt);    %[] Compute joint states at next timestep

        %% UPDATE : BCS Guess & History
        F_0 = Update_F_0(Robot);
        Robot = Update_Joints(Robot,Theta,Theta_dot,Theta_ddot(:,ii));

        %% ANIMATE :
        if Animate
            [imind,cm,g_EE] = Animate_Grasp(Robot,Theta,NumStep,NumBod,PlotObj,Fig);
            EE_POS(:,:,data_index) = g_EE(1:3,4,:);
            if i == 1 && ii == 1
                imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
            else 
                imwrite(imind,cm,filename,'gif','DelayTime',dt,'WriteMode','append'); 
            end 
            if data_index == SNAP(snap_count)
                Robo_SNAP{snap_count} = Robot;
                snap_count = snap_count + 1;
            end
        end
        
        %% CONTACT DYNAMICS :
        % Spring and Damping Coefficients
        k = 3e2;    % Mod Traj w/ Slow Contact
        c = 1.5e-1;
        
        % Determine if in contact with Target (symm. both sides)
        if norm(EE_POS(:,1,data_index) - [0;-Target_Offset;.2]) < Target_R
%             disp('!! TOUCHING !!')
            Diff_1 = (EE_POS(:,1,data_index) - [0;-Target_Offset;Target_R]);
            Diff_1 = Diff_1 .* (1-Target_R/norm(Diff_1));
            Diff_2 = (EE_POS(:,2,data_index) - [0;-Target_Offset;Target_R]);
            Diff_2 = Diff_2 .* (1-Target_R/norm(Diff_2));
            
            F_ext(1:3,1) =  - k*Diff_1 - c*Robot{end-2}.eta_prev(1:3,end);
            F_ext(1:3,2) =  - k*Diff_2 - c*Robot{end-8}.eta_prev(1:3,end);
            F_ext(1:3,1) =  g_EE(1:3,1:3,1) * F_ext(1:3,1);
            F_ext(1:3,2) =  g_EE(1:3,1:3,2) * F_ext(1:3,2);
            
            F_H(:,data_index) = F_ext(1:3,2);
        end
    end
end
% toc
% xlabel('X')
% ylabel('Y')
% zlabel('Z')

%% PLOTS 
Ln_Clr  = {'r',[.5,.5,.5],'k','r',[.5,.5,.5],'k','r',[.5,.5,.5],'k'};
Ln_Styl = {'--','--','--','-','-','-',':',':',':'};
num_PhaseI  = sum(TimeStep(1:5)); 
num_PhaseII = num_PhaseI + 1;
    
DEBUG = 1;
TOGG_PLOT = 0;
if DEBUG 
    %% Plot Tip Force
    figure()
    for i = 1:3
        plot(t,F_H(i,:),'LineWidth',2,'LineStyle',Ln_Styl{i+3},'Color',Ln_Clr{i+3}),hold on, grid on
    end
    title("k=" + k + ", c=" + c)
    xlabel('Time(s)')
    ylabel('Tip Force')
    legend('X','Y','Z')
end
if TOGG_PLOT
    %% Plot Joint Actuation
    % For all time
    figure()
    for i = 1:NumBod
        plot(t,C(i,:),'LineWidth',2,'LineWidth',2,'LineStyle',Ln_Styl{i},'Color',Ln_Clr{i}),hold on, grid on
    end
    xlabel('Time(s)')
    ylabel('Joint Act (Nm or N)')

    figure()    
    % Plot Segment I
    for i = 1:NumBod
        plot(t(1:num_PhaseI),C(i,1:num_PhaseI),'LineWidth',2,'LineStyle',Ln_Styl{i},'Color',Ln_Clr{i}),hold on, grid on
    end
    xlabel('Time (s)')
    ylabel('Joint Act (Nm or N)')
    legend('T_1','T_2','T_3','T_4','T_5','T_6','T_7')
    
    %% Plot Joint Angles
    % For all time
    figure()
    for i = 1:NumBod
        plot(t,T_H(i,:),'LineWidth',2,'LineStyle',Ln_Styl{i},'Color',Ln_Clr{i}),hold on, grid on
    end
    xlabel('Time(s)')
    ylabel('Joint Angle(rad)')
    
    % For Phase blender I
    figure()
    for i = 1:NumBod
        plot(t(1:num_PhaseI),T_H(i,1:num_PhaseI),'LineWidth',2,'LineStyle',Ln_Styl{i},'Color',Ln_Clr{i}),hold on, grid on
    end
    xlabel('Time(s)')
    ylabel('Joint Angle(rad)')
    legend('\theta_1','\theta_2','\theta_3','\theta_4','\theta_5','\theta_6','\theta_7')

    %% Plot EE Location
    figure()
    % Plot all
    for i = 1:3
        plot(t,reshape(EE_POS(i,1,:),[1,sum(TimeStep)]),'LineWidth',2,'LineStyle',Ln_Styl{i},'Color',Ln_Clr{i}),hold on, grid on
    end
    for i = 1:3
        plot(t,reshape(EE_POS(i,2,:),[1,sum(TimeStep)]),'LineWidth',2,'LineStyle',Ln_Styl{i+3},'Color',Ln_Clr{i+3}),hold on, grid on
    end
    xlabel('Time (s)')
    ylabel('EE Position (m)')
    legend('X_1','Y_1','Z_1','X_2','Y_2','Z_2')
    
    figure()    
    % Plot Segment I
    for i = 1:3
        plot(t(1:num_PhaseI),reshape(EE_POS(i,1,1:num_PhaseI),[1,num_PhaseI]),'LineWidth',2,'LineStyle',Ln_Styl{i},'Color',Ln_Clr{i}),hold on, grid on
    end
    for i = 1:3
        plot(t(1:num_PhaseI),reshape(EE_POS(i,2,1:num_PhaseI),[1,num_PhaseI]),'LineWidth',2,'LineStyle',Ln_Styl{i+3},'Color',Ln_Clr{i+3}),hold on, grid on
    end
    xlabel('Time (s)')
    ylabel('EE Position (m)')
    legend('X_1','Y_1','Z_1','X_2','Y_2','Z_2')
    
    %% Plot Tip Force
    figure()
    for i = 1:3
        plot(t,F_H(i,:),'LineWidth',2,'LineStyle',Ln_Styl{i+3},'Color',Ln_Clr{i+3}),hold on, grid on
    end
    title("k=" + k + ", c=" + c)
    xlabel('Time(s)')
    ylabel('Tip Force')
    legend('X','Y','Z')
    
    %% Plot System Pose
    % Initialize
    Plot_SnapShot(SNAP,Robot,Robo_SNAP,T_H,PlotObj,VisScale,NumStep)
    
end

%% AUXILARY FUNCTIONS
function Robot = Update_Joints(Robot,Theta,Theta_dot,Theta_ddot)
% Apply joint Pos, Vel, Acc to corresponding objects in the Robot structure

    NumBod = 7;
    for ii = 1:NumBod-2
        Robot{2*ii}.Position = Theta(ii);
        Robot{2*ii}.Vel = Theta_dot(ii);
        Robot{2*ii}.Accel = Theta_ddot(ii);
    end
    for ii = NumBod - [1,0]
        Robot{2*ii+2}.Position = Theta(ii);
        Robot{2*ii+2}.Vel = Theta_dot(ii);
        Robot{2*ii+2}.Accel = Theta_ddot(ii);
    end
end
function [Theta,Theta_New,Theta_dot_New] = Step_Joints(Theta,Theta_dot,Theta_ddot,dt)
% Step joint angles in time assuming Euler Integration
    Theta_New = Theta + Theta_dot*dt;
    Theta_dot_New = Theta_dot + Theta_ddot*dt;
end
function F_0 = Update_F_0(Robot)
% Assume previous solution for each flexible body as the initial guess 
    BC_Start = 5;
    F_0(:,1) = Robot{2*BC_Start - 1}.f_prev(:,1);
    F_0(:,2) = Robot{2*(BC_Start+3) - 1}.f_prev(:,1);
end
function Plot_SnapShot(SNAP,Robot,Robo_SNAP,Theta,PlotObj,VisScale,NumStep)
    figure(24);
    NumBod = 7;
    view(3), axis equal, axis(VisScale*[-1 1 -1.5 .2 -.01 2]), grid on, Axes = gca;
    
    % Define Cell Array to Store Handle for each Body
    for j = 1:length(SNAP) - 1
        for i = 1:NumBod - 2                            %[]     Make Handle for each Body
            if strcmp(Robot{2*i+1}.Type,'FLEXIBLE')     %[]     Linestyle for Flexible Bodies
                PlotObj_Snap{j}{i} = animatedline(Axes,'Color','r','LineStyle','none','Marker','.','MarkerSize',10);
            elseif strcmp(Robot{2*i+1}.Type,'RIGID')    %[]     Linestyle for Rigid Bodies
                PlotObj_Snap{j}{i} = animatedline(Axes,'Color','k','LineStyle','-','LineWidth',3,'Marker','.');
            end
        end
        for i = NumBod - [1,0]                            %[]     Make Handle for each Body
            if strcmp(Robot{2*i+3}.Type,'FLEXIBLE')     %[]     Linestyle for Flexible Bodies
                PlotObj_Snap{j}{i} = animatedline(Axes,'Color','r','LineStyle','none','Marker','.','MarkerSize',10);
            elseif strcmp(Robot{2*i+3}.Type,'RIGID')    %[]     Linestyle for Rigid Bodies
                PlotObj_Snap{j}{i} = animatedline(Axes,'Color','k','LineStyle','-','LineWidth',3,'Marker','.');
            end
        end
    end
    
    for i = 1:length(SNAP)-1
        [~,POS_R,POS_F] = Pos_MB_RE(Robo_SNAP{i},Theta(:,SNAP(i)),NumStep);
        Start_F = 1;        % Record the index in POS_F
        Count_R = 1;        % Record the index in POS_R
        
        for ii = 1:NumBod-2
            clearpoints(PlotObj{ii})
            if strcmp(Robot{2*ii+1}.Type,'FLEXIBLE')        % Linestyle for Flexible Bodies
                N_Body = Robot{2*ii+1}.N * NumStep;         % Determine number of data points in body
                End_F = Start_F + N_Body - 1;               % Identify end index of the body
                addpoints(PlotObj_Snap{i}{ii},POS_F(1,Start_F:End_F),POS_F(2,Start_F:End_F),POS_F(3,Start_F:End_F));
                Start_F = End_F + 1;
            elseif strcmp(Robot{2*ii+1}.Type,'RIGID')        % Linestyle for Rigid Bodies
                addpoints(PlotObj_Snap{i}{ii},POS_R(1,Count_R:Count_R+1),POS_R(2,Count_R:Count_R+1),POS_R(3,Count_R:Count_R+1));
                Count_R = Count_R + 2;
            end
        end
        for ii = NumBod - [1,0]
            clearpoints(PlotObj{ii})
            if strcmp(Robot{2*ii+3}.Type,'FLEXIBLE')        % Linestyle for Flexible Bodies
                N_Body = Robot{2*ii+3}.N * NumStep;         % Determine number of data points in body
                End_F = Start_F + N_Body - 1;               % Identify end index of the body
                addpoints(PlotObj_Snap{i}{ii},POS_F(1,Start_F:End_F),POS_F(2,Start_F:End_F),POS_F(3,Start_F:End_F));
                Start_F = End_F + 1;
            elseif strcmp(Robot{2*ii+3}.Type,'RIGID')        % Linestyle for Rigid Bodies
                addpoints(PlotObj_Snap{i}{ii},POS_R(1,Count_R:Count_R+1),POS_R(2,Count_R:Count_R+1),POS_R(3,Count_R:Count_R+1));
                Count_R = Count_R + 2;
            end
        end
        drawnow, pause(.05)
    end
    drawnow
end