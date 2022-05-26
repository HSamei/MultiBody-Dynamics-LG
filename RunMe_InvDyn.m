%% %%%%%%%%%%%%% Rigid Open-Chain Multi-Body Inverse Dynamic Calculator %%%%%%%%%%%%% %%
%#ok<*CCAT>
% CAN DEFINE TRANSFORMATIONS INTO JOINT OR BODY. CHOOSE JOINTS SO CAN EASILY
% IDENTIFY THE BODY FRAME @ COM AND JOINT FRAME IS NOT AS USEFUL TO TRACK.
% 
% Written by BD Bhai

%% LOAD LIBRARY
addpath(strcat(pwd,'/Library'))

%% SIMULATION PARAMETERS
%% Param for defPaperSample_1()
% Theta       = [0;0;0];      %[rad]      Actuated Joint Position
% Theta_dot   = [0;0;0];      %[rad/s]    Actuated Joint Velocity
% 
% dt          = .025;         %[s]        Time Step Size
% TimeStep    = 10;           %[]         Number of Time Steps Evaluated
% VisScale    = 1.2;          %[]         Scale to Axes Dimensions if System Out of Plot View
% NumStep     = 2;            %[]         Factor to shrink ds for Plotting Flexible Bodies
% RestTime    = 5;            %[]         Hold Joint Rates at 0 for RestTime*TimeStep*dt
% 
% t1 = dt:dt:dt*TimeStep;                             %[s]    Time Vector 1
% t2 = dt*TimeStep+dt:dt:2*dt*TimeStep;               %[s]    Time Vector 2
% t3 = 2*dt*TimeStep+dt:dt:(2+RestTime)*dt*TimeStep;  %[s]    Time Vector 3
% 
% % Define Trajectory over time
% Shape       = [6;-4;-6];                                %[rad/s2]   Actuated Joint Acceleration Shape
% Theta_ddot1 = Shape .* ones(1,TimeStep);                %[rad/s2]   Joint Trajectory 1
% Theta_ddot2 = -Shape.* ones(1,TimeStep);                %[rad/s2]   Joint Trajectory 2
% Theta_ddot3 = 0 .* Shape .* ones(1,TimeStep*RestTime);  %[rad/s2]   Joint Trajectory 3
% Theta_ddot  = [Theta_ddot1, Theta_ddot2, Theta_ddot3];  %[rad/s2]   Joint Trajectory Combined
% 
% F_ext       = [0;0;0;0;0;0];        %[N;Nm] Applied Wrench at the EE
% F_0         = [0;0;1;0;0;0];        %[]     Free Strain in first Flexible Body
% filename    = 'TestPaper_1_Inv.gif';    %[]     Filename for Output Video
% SNAP        = [1,22,33,0];          %[]     TimeStep to Plot Snapshot (ascending order and padd with 0)
% saveControl = 1;                    %[]     Save Control Torque to Output File (MODIFY FILENAME)
% saveName    = 'ControlSim1';        %[]     Output Filename for Control Torque
% Robot       = defPaperSample_1(Theta,Theta_dot,Theta_ddot(:,1));
%% Param for defPaperSample_2()
Theta       = [0;0;0;0;0];  %[rad]      Actuated Joint Position
Theta_dot   = [0;0;0;0;0];  %[rad/s]    Actuated Joint Velocity

dt          = .025;         %[s]    Time Step Size
TimeStep    = 100;          %[]     Number of Time Steps Evaluated
VisScale    = 2;            %[]     Scale to Axes Dimensions
NumStep     = 2;            %[]     Factor to shrink ds for Plotting Flexible Bodies
RestTime    = 0;            %[]     Hold Joint Rates at 0 for RestTime*TimeStep*dt

t1 = dt:dt:dt*TimeStep;                             %[s]    Time Vector 1
% Define Trajectory over time
Shape       = .4*[1;-.5;.5;-.7;-.1];                %[rad/s2]   Actuated Joint Acceleration Shape
Theta_ddot = Shape .* sin(pi/(dt*TimeStep).*t1);    %[rad/s2]   Joint Trajectory

F_ext       = [0;0;0;0;0;0];        %[N;Nm] Applied Wrench at the EE
F_0         = [0;0;1;0;0;0];        %[]     Free Strain in first Flexible Body
filename    = 'TestPaper_2_Inv.gif';    %[]     Filename for Output Video
SNAP        = [1,100,0];            %[]     TimeStep to Plot Snapshot (ascending order and pad with 0)
saveControl = 1;                    %[]     Save Control Torque to Output File
saveName    = 'ControlSim2';        %[]     Output Filename for Control Torque
Robot = defPaperSample_2(Theta,Theta_dot,Theta_ddot(:,1));

%% INITIALIZATION FOR RESPONSE LOOP
[BC_Start,BC_End,NumBod] = MB_Flex_ID(Robot,Theta,Theta_dot,Theta_ddot(:,1));     %[]     Number of Bodies & ID Flexible
t = dt:dt:dt*length(Theta_ddot(1,:));   %[s]    Time Vector
C = zeros(NumBod,length(t));            %[N,Nm] Allocate Memory to save the Control Torques
T_H  = zeros(NumBod,length(t));         %[N,Nm] Allocate Memory to save the Joint Angles
Td_H = zeros(NumBod,length(t));         %[N,Nm] Allocate Memory to save the Joint Velocity
EE_POS = zeros(3,length(t));            %[N,Nm] Allocate Memory to save the EE Position
Robo_SNAP{length(SNAP)} = 0;            %[]     Allocate Memory to save the Robot States for SNAP
PrevGuess = 1;                          %[]     Use Previous Guess (1) or BDF-2 Mod Guess (0)
Animate = 1;                            %[]     Generate Animation Video and File (1)
snap_count = 1;                         %[]     Counter for Saving the Snapshots Data

% Initialization of Objects for Animation
if Animate
    Fig = figure(21);
    view(3), axis equal, axis(VisScale*[-1 1 -1 1 -.01 1]), grid on, Axes = gca;

    % Define Cell Array to Store Handle for each Body
    PlotObj{NumBod} = 0;                        %[]     Initialize Cell Array Size1
    for i = 1:NumBod                            %[]     Make Handle for each Body
        if strcmp(Robot{2*i+1}.Type,'FLEXIBLE')         % Linestyle for Flexible Bodies
            PlotObj{i} = animatedline(Axes,'Color','r','LineStyle','none','Marker','.','MarkerSize',10);
        elseif strcmp(Robot{2*i+1}.Type,'RIGID')        % Linestyle for Rigid Bodies
            PlotObj{i} = animatedline(Axes,'Color','k','LineStyle','-','LineWidth',3,'Marker','.');
        end
    end
end

%% RUN LOOP FOR TIME RESPONSE
for i = 1:length(t)
    %% UPDATE : For next time step
    [F,C(:,i),eta,Robot_New] = IDM_MB_RE(Robot, Theta, Theta_dot, Theta_ddot(:,i), F_ext, dt, F_0);   %[] Save strain to each body in Robot Struct
    
    % Update initial strain guess for next iteration
    if  BC_End > BC_Start                   %[]     Only if are flexible members
        FlexBod = Robot{2*BC_Start-1};
        FlexBod_New = Robot{2*BC_Start-1};
        if PrevGuess == 1                   %[]     Assume current state as guess for next 
            F_0 = FlexBod_New.f_prev(:,1);
        else
            F_0 = 2.5*FlexBod_New.f_prev(:,1) - 2*FlexBod.f_prev(:,1) + .5*FlexBod.f_pprev(:,1);
        end
    end
    Robot = Robot_New;      %[] Delay update of Structure to use strain data at current, previous and pprevious iterations
    
    % Joint Values Updated (Assumes Euler Integration and Theta_ddot constant for now)
    T_H(:,i) = Theta;
    Td_H(:,i) = Theta_dot;
    
    Theta = Theta + Theta_dot*dt;
    Theta_dot = Theta_dot + Theta_ddot(:,i)*dt;
    % Update Joints
    for ii = 1:NumBod
        Robot{2*ii}.Position = Theta(ii);
        Robot{2*ii}.Vel = Theta_dot(ii);
        Robot{2*ii}.Accel = Theta_ddot(ii,i);
    end
    
    %% ANIMATE :
    if Animate
        [imind,cm,EE_POS(:,i)] = Animate_MB_RE(Robot,Theta,NumStep,NumBod,PlotObj,Fig);
        if i == 1
            imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
        else 
            imwrite(imind,cm,filename,'gif','DelayTime',dt,'WriteMode','append'); 
        end
        if i == SNAP(snap_count)
            Robo_SNAP{snap_count} = Robot;
            snap_count = snap_count + 1;
        end
    end
end

%% POST-PROCESSING
TOGG_PLOT = 1;
if TOGG_PLOT
    %% Save C_des
    if saveControl
        C_des = C;
        save(saveName,'C_des')  %[] Save to Ooutput File
    end
    
    %% Plot Joint Actuation
    figure()
    for i = 1:NumBod
        plot(t,C(i,:)),hold on, grid on
    end
    xlabel('Time(s)')
    ylabel('Joint Act (Nm or N)')

    %% Plot Joint Angles
    figure()
    for i = 1:NumBod
        plot(t,T_H(i,:)),hold on, grid on
    end
    xlabel('Time(s)')
    ylabel('Joint Angle')

    %% Plot EE Location
    figure()
    for i = 1:NumBod
        plot(t,EE_POS(1,:)),hold on, grid on
    end
    xlabel('Time(s)')
    ylabel('EE Position')
    
    %% Plot System Pose
    % Initialize
    Fig = figure(24);
    view(3), axis equal, grid on, Axes = gca;

    % Define Cell Array to Store Handle for each Body
    for j = 1:length(SNAP)-1
        for i = 1:NumBod                                %[]     Make Handle for each Body
            if strcmp(Robot{2*i+1}.Type,'FLEXIBLE')     %[]     Linestyle for Flexible Bodies
                PlotObj_Snap{j}{i} = animatedline(Axes,'Color','r','LineStyle','none','Marker','.','MarkerSize',10);
            elseif strcmp(Robot{2*i+1}.Type,'RIGID')    %[]     Linestyle for Rigid Bodies
                PlotObj_Snap{j}{i} = animatedline(Axes,'Color','k','LineStyle','-','LineWidth',3,'Marker','.');
            end
        end
    end
    
    for i = 1:length(SNAP)-1
        [POS,POS_R,POS_F] = Pos_MB_RE(Robo_SNAP{i},T_H(:,SNAP(i)),NumStep);
        Start_F = 1;        % Record the index in POS_F
        Count_R = 1;        % Record the index in POS_R
        
        for ii = 1:NumBod
            clearpoints(PlotObj_Snap{i}{ii})
            if strcmp(Robo_SNAP{i}{2*ii+1}.Type,'FLEXIBLE')        % Linestyle for Flexible Bodies
                N_Body = Robo_SNAP{i}{2*ii+1}.N * NumStep;         % Determine number of data points in body
                End_F = Start_F + N_Body - 1;                      % Identify end index of the body
                addpoints(PlotObj_Snap{i}{ii},POS_F(1,Start_F:End_F),POS_F(2,Start_F:End_F),POS_F(3,Start_F:End_F));
                Start_F = End_F + 1;
            elseif strcmp(Robo_SNAP{i}{2*ii+1}.Type,'RIGID')        % Linestyle for Rigid Bodies
                addpoints(PlotObj_Snap{i}{ii},POS_R(1,Count_R:Count_R+1),POS_R(2,Count_R:Count_R+1),POS_R(3,Count_R:Count_R+1));
                Count_R = Count_R + 2;
            end
        end
    end
    drawnow
end