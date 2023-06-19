function Robot = defMultiBranch_Paper_Test(Theta,Theta_dot,Theta_ddot)
%#ok<*CCAT>
% RefModel = importrobot('iiwa14.urdf');    

    %% DEFINE THE GEOMETRY OF THE COMPONENTS
    % LINK PARAMETERS (Rigid Links)
    m   = .01;                     %[kg]       Mass of Link
    L   = .2;                       %[m]        Length of Link
    L_z = L*[0;0;1;0;0;0];          %[se(3)]    Twist associated with Body/Link Transformation, Begin to End
    r_z = L_z/2;                    %[se(3)]    Twist associated with CoM Transformation, Begin to CoM
    
    I  = 1/12*m*L^2*eye(3);         %[kg m2]    Moment of Inertia of Body about CoM in Body Coordinate Frame (NEED TO FIX SLENDER AXIS)
    M  = [ m.*eye(3), zeros(3)  ;...
            zeros(3),       I  ];   %[kg;kg m2] Mass Matrix for Links (assumed same so far but can modify later)
    
    % BEAM PARAMETERS (Thin Rods)
    L_0 = .2;            %[m]        Free Length of the Modeled Beam
    F_0 = [0;0;1;0;0;0]; %[]         Free Static Strain under no Applied Loads 
    rho = 75e1;          %[kg/m3]    Density of Material
    mu  = 2e4;           %[N/m^2s]   Viscosity of Peanut Butter
%     mu  = 0;           %[N/m^2s]   Viscosity of Peanut Butter
    r = .01;             %[m]        Radius of Cross-Section
    E = 2e8;             %[Pa]       Youngs Modulus
    G = E/(2*(1+.3));    %[Pa]       Shear Modulus
    A = pi*r^2;          %[m2]       Cross-Sectional Area of Beam
    I = pi/4*r^4;        %[m4]       2nd Moment of Inertia of Beam
    N = 21;              %[]         Number of Discretized Elements
    
    J = diag([2*I,I,I]);            %[m4]       3D Moment of Inertia of Beam
    Kbt = diag([2*G*I,E*I,E*I]);    %[Nm2]      Bending and Torsional Rigidity (Rotational)
    Kse = diag([E*A,G*A,G*A]);      %[N]        Shear and Extension Rigidity (Linear)
    Cse = diag([3*A,A,A])*mu;       %[N/s]      Shear and Extension Damping (Linear)
    Cbt = diag([2*I,I,I])*mu;       %[N/s]      Bending and Torsional Damping (Rotational)
    
    K = [Kse,zeros(3);zeros(3),Kbt];
    C = [Cse,zeros(3);zeros(3),Cbt];
%     C = K .* 0;
    Mf = [rho*A*eye(3),zeros(3);zeros(3),rho*J];
    Z = zeros(6,1);
    
    %% DEFINE THE MANIPULATOR
    % Define Bodies
    BASE   = Body('BASE'  ,'RIGID',Inf*eye(6),Z,Z);
    Body_1 = Body('Body_1','RIGID',M,L_z,r_z);
    Body_2 = Body('Body_2','RIGID',M,L_z,r_z);
    Body_3 = Body('Body_3','RIGID',M,L_z,r_z);              %[]     Split into 2 Sections
        Body_3.Transform_1 = Body_3.Transform;              %[]     Connect to Branch_1
        Body_3.Transform_2 = Body_3.Transform;              %[]     Connect to Branch_2
    Body_4 = Body('Body_4','FLEXIBLE',Mf,K,C,F_0,N,L_0);
    Body_5 = Body('Body_5','FLEXIBLE',Mf,K,C,F_0,N,L_0);
    Body_6 = Body('Body_6','FLEXIBLE',Mf,K,C,F_0,N,L_0);
    Body_7 = Body('Body_7','FLEXIBLE',Mf,K,C,F_0,N,L_0);
    EE_1   = Body('EE_1'  ,'RIGID',zeros(6),Z,Z);
    EE_2   = Body('EE_2'  ,'RIGID',zeros(6),Z,Z);
    
    % Give Previous States for Flexible Bodies
    Body_4.eta_prev  = zeros(6,Body_4.N);
    Body_4.eta_pprev = zeros(6,Body_4.N);
    Body_4.f_prev    = ones(1,Body_4.N) .* Body_4.F_0;
    Body_4.f_pprev   = ones(1,Body_4.N) .* Body_4.F_0;
    Body_5.eta_prev  = zeros(6,Body_5.N);
    Body_5.eta_pprev = zeros(6,Body_5.N);
    Body_5.f_prev    = ones(1,Body_5.N) .* Body_5.F_0;
    Body_5.f_pprev   = ones(1,Body_5.N) .* Body_5.F_0;
    Body_6.eta_prev  = zeros(6,Body_6.N);
    Body_6.eta_pprev = zeros(6,Body_6.N);
    Body_6.f_prev    = ones(1,Body_6.N) .* Body_6.F_0;
    Body_6.f_pprev   = ones(1,Body_6.N) .* Body_6.F_0;
    Body_7.eta_prev  = zeros(6,Body_7.N);
    Body_7.eta_pprev = zeros(6,Body_7.N);
    Body_7.f_prev    = ones(1,Body_7.N) .* Body_7.F_0;
    Body_7.f_pprev   = ones(1,Body_7.N) .* Body_7.F_0;
    
    % Define Joints       {Screw axis here in joint frame}
    Joint_1 = Joint('Joint_1','RIGID',[0;0;0;0;0;1],Theta(1),Theta_dot(1),Theta_ddot(1),[-pi,pi],0,BASE,Body_1);
    Joint_2 = Joint('Joint_2','RIGID',[0;0;0;1;0;0],Theta(2),Theta_dot(2),Theta_ddot(2),[-pi,pi],pi/2,Body_1,Body_2);
    Joint_3 = Joint('Joint_3','RIGID',[0;0;0;1;0;0],Theta(3),Theta_dot(3),Theta_ddot(3),[-pi,pi],0,Body_2,Body_3);
    Joint_4 = Joint('Joint_4','RIGID',[0;0;0;0;1;0],Theta(4),Theta_dot(4),Theta_ddot(4),[-pi,pi],pi/2,Body_3,Body_4);
    Joint_5 = Joint('Joint_5','RIGID',[0;0;0;0;1;0],Theta(5),Theta_dot(5),Theta_ddot(5),[-pi,pi],-pi/4,Body_4,Body_5);
    Joint_6 = Joint('Joint_6','RIGID',[0;0;0;0;1;0],Theta(6),Theta_dot(6),Theta_ddot(6),[-pi,pi],-pi/2,Body_3,Body_6);
    Joint_7 = Joint('Joint_7','RIGID',[0;0;0;0;1;0],Theta(7),Theta_dot(7),Theta_ddot(7),[-pi,pi],pi/4,Body_6,Body_7);
    
    % Define Robot combining the Bodies and Joints {'BASE', Joint, Body, ... , Joint, Body, 'EE'}
    Robot = { BASE    , Joint_1, Body_1 };
    Robot = { Robot{:}, Joint_2, Body_2 };
    Robot = { Robot{:}, Joint_3, Body_3 };      %[]    Rigid Sub-section
    
    Robot = { Robot{:}, Joint_4, Body_4 };
    Robot = { Robot{:}, Joint_5, Body_5 };
    Joint_EE = Joint('Joint_EE_1','RIGID',zeros(6,1),0,0,0,0,0,Robot{end},EE_1);    %[Joint]    Define a 'joint' that map CoM last Body to EE
    Robot = { Robot{:}, Joint_EE, EE_1 };       %[]    Flexible Branch_1
    
    Robot = { Robot{:}, Joint_6, Body_6 };
    Robot = { Robot{:}, Joint_7, Body_7 };
    Joint_EE = Joint('Joint_EE_2','RIGID',zeros(6,1),0,0,0,0,0,Robot{end},EE_2);    %[Joint]    Define a 'joint' that map CoM last Body to EE
    Robot = { Robot{:}, Joint_EE, EE_2 };       %[]    Flexible Branch_2
end