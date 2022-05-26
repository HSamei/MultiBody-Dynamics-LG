%% %%%%%%%%%%%%% Rigid Open-Chain Multi-Body Forward Dynamics Calculator %%%%%%%%%%%%% %%
% Written by BD Bhai
%#ok<*CCAT>
% CAN DEFINE TRANSFORMATIONS INTO JOINT OR BODY. CHOOSE JOINTS SO CAN EASILY
% IDENTIFY THE BODY FRAME @ COM AND JOINT FRAME IS NOT AS USEFUL TO TRACK
% 

%% Initialize System
addpath(strcat(pwd,'/Library'))

%% SIMULATION PARAMETERS (uncomment desired simulation)
%% Param for defPaperSample_1() (UNSTABLE AND DOESN'T WORK)
% Theta       = [0;0;0];      %[]     Actuated Joint Position
% Theta_dot   = [0;0;0];      %[]     Actuated Joint Velocity
% InitGuess   = [6;-4;-6];   %[]     Guessed Joint Accelerations
% 
% % Load Control Input from file or define here as C_des as vector over all timesteps
% load('ControlSim1')              %[N;Nm]     Load C_des from InvKin Solution (#Joints X #TimeSteps)
% 
% dt          = .025;         %[s]    Time Step Size
% VisScale    = 1.2;            %[]     Scale to Axes Dimensions
% NumStep     = 2;            %[]     Factor to shrink ds for Flexible Bodies
% 
% F_ext       = [0;0;0;0;0;0];    %[N;Nm] Applied Wrench at the EE
% F_0         = [0;0;1;0;0;0];    %[]     Free Strain in first Flexible Body
% Robot = defPaperSample_1(Theta,Theta_dot,InitGuess);
% filename = 'TestPaper_1_Fwd.gif';
%% Param for defPaperSample_2()
Theta       = [0;0;0;0;0];  %[rad]      Actuated Joint Position
Theta_dot   = [0;0;0;0;0];  %[rad/s]    Actuated Joint Velocity
dt          = .025;         %[s]        Time Step Size
VisScale    = 2;            %[]         Scale to Axes Dimensions
NumStep     = 2;            %[]         Factor to shrink ds for Flexible Bodies in Plotting

% Load Control Input from file or define here as C_des as vector over all timesteps
InitGuess = [0;0;0;0;0];    %[rad/s2]   Guessed Joint Accelerations
load('ControlSim2')         %[N;Nm]     Load C_des from InvKin Solution (#Joints X #TimeSteps)

F_ext       = [0;0;0;0;0;0];        %[N;Nm]     Applied Wrench at the EE
F_0         = [0;0;1;0;0;0];        %[]         Free Strain in first Flexible Body
filename    = 'TestPaper_2_Fwd.gif';  %[]         Filename for Output Video
Robot = defPaperSample_2(Theta,Theta_dot,InitGuess);

%% SETUP & INITIALIZATION FOR RESPONSE LOOP
[BC_Start,BC_End,NumBod] = MB_Flex_ID(Robot,Theta,Theta_dot,C_des(:,1));     %[]     Number of Bodies & ID Flexible
t = dt:dt:dt*length(C_des(1,:));    %[s]   Time Vector
T_H = zeros(NumBod,length(t));      %[rad] Allocate Memory to save the Joint Angles
Tdd_H = zeros(NumBod,length(t));    %[rad] Allocate Memory to save the Joint Accelerations
C = zeros(NumBod,length(t));        %[Nm]  Allocate Memory to save the Control Actuations
% Initialize Animation
Fig = figure(21);
% Make Object for each Body
for i = 1:NumBod
    if strcmp(Robot{2*i+1}.Type,'FLEXIBLE')         % Linestyle for Flexible Bodies
        PlotObj{i} = animatedline('Color','r','LineStyle','none','Marker','.','MarkerSize',10);
    elseif strcmp(Robot{2*i+1}.Type,'RIGID')        % Linestyle for Rigid Bodies
        PlotObj{i} = animatedline('Color','k','LineStyle','-','LineWidth',3,'Marker','.');
    end
end
% Counters and parameters for Plotting MB
view(3), axis equal, axis(VisScale*[-1 1 -1 1 -.01 1]), grid on, Axes = gca;

%% RUN LOOP FOR TIME RESPONSE
for i = 1:length(t)
    %% UPDATE : 
    % Compute System Dynamics
    [F,Joint_Acc,C_curr,Robot] = FDM_MB_RE(Robot, Theta, Theta_dot, C_des(:,i), F_ext, dt, F_0, InitGuess);   
    % Save Data for Post-Processing
    T_H(:,i)   = Theta;
    Tdd_H(:,i) = Joint_Acc;
    C(:,i)     = C_curr;
    
    % BCS Initial Guesses
    InitGuess = Joint_Acc;       %[]    Update Joint Acceleration guess
    if  BC_End > BC_Start        %[]    Update Strain guess if there are any flexible members
        F_0 = Robot{2*BC_Start-1}.f_prev(:,1);
    end
    
    % Update Joints (Pos, Vel, Acc) using Euler Integration 
    Theta_ddot = Joint_Acc;
    Theta_dot  = Theta_ddot*dt + Theta_dot;
    Theta      = Theta_dot*dt + Theta ;
    for ii = 1:NumBod
        Robot{2*ii}.Position = Theta(ii);
        Robot{2*ii}.Vel = Theta_dot(ii);
        Robot{2*ii}.Accel = Theta_ddot(ii);
    end    
    %% ANIMATE : Add frame to output '.gif' file
    [imind,cm] = Animate_MB_RE(Robot,Theta,NumStep,NumBod,PlotObj,Fig);
    if i == 1
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
    else 
        imwrite(imind,cm,filename,'gif','DelayTime',dt,'WriteMode','append'); 
    end     
end

%% POST-PROCESSING: Plot Joint Angles, Accelerations, Actuations, etc.
TOGG_PLOT = 1;
if TOGG_PLOT
    % Plot Joint Actuation
    figure()
    for i = 1:NumBod
        plot(t,C(i,:)),hold on, grid on
    end
    xlabel('Time(s)')
    ylabel('Joint Act(Nm or N)')

    % Plot Joint Angles
    figure()
    for i = 1:NumBod
        plot(t,T_H(i,:)),hold on, grid on
    end
    xlabel('Time(s)')
    ylabel('Joint Angle(rad)')
end