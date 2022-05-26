function Robot = defPaperSample_2(Theta,Theta_dot,Theta_ddot)
%#ok<*CCAT>
    
    %% DEFINE THE GEOMETRY OF THE COMPONENTS
    % LINK PARAMETERS (Rigid Links)
    m   = 12.5;                     %[kg]       Mass of Link
    L   = .5;                       %[m]        Length of Link
    L_z = L*[0;0;1;0;0;0];          %[se(3)]    Twist associated with Body/Link Transformation, Begin to End
    r_z = L_z/2;                    %[se(3)]    Twist associated with CoM Transformation, Begin to CoM
    
    I  = 1/12*m*L^2*eye(3);         %[kg m2]    Moment of Inertia of Body about CoM in Body Coordinate Frame (NEED TO FIX SLENDER AXIS)
    M = [ m.*eye(3), zeros(3)  ;...
           zeros(3),       I  ];    %[kg;kg m2] Mass Matrix for Links (assumed same so far but can modify later)
    
    % BEAM PARAMETERS (Thin Rods)
    L_0 = .4;            %[m]        Free Length of the Modeled Beam
    F_0 = [0;0;1;0;0;0]; %[]         Free Static Strain under no Applied Loads 
    rho = 75e1;           %[kg/m3]    Density of Material
    mu  = 1e5;           %[N/m^2s]   Viscosity of Peanut Butter
    r = .1;              %[m]        Radius of Cross-Section
    E = 1e9;             %[Pa]       Youngs Modulus
    G = E/(2*(1+.3));    %[Pa]       Shear Modulus
    A = pi*r^2;          %[m2]       Cross-Sectional Area of Beam
    I = pi/4*r^4;        %[m4]       2nd Moment of Inertia of Beam
    N = 21;              %[]         Number of Discretized Elements
      
    J = diag([2*I,I,I]);            %[m4]       3D Moment of Inertia of Beam
    Kbt = diag([2*G*I,E*I,E*I]);    %[Nm^2]     Bending and Torsional Rigidity (Rotational)
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
    Body_2 = Body('Body_2','FLEXIBLE',Mf,K,C,F_0,N,L_0);
    Body_3 = Body('Body_3','RIGID',M,L_z,r_z);
    Body_4 = Body('Body_4','FLEXIBLE',Mf,K,C,F_0,N,L_0);
    Body_5 = Body('Body_5','RIGID',M/2,L_z/2,r_z/2);
    EE     = Body('EE'    ,'RIGID',zeros(6),Z,Z);
    
    % Give Previous States for Flexible Bodies
    Body_2.eta_prev  = zeros(6,Body_2.N);
    Body_2.eta_pprev = zeros(6,Body_2.N);
    Body_2.f_prev    = ones(1,Body_2.N) .* Body_2.F_0;
    Body_2.f_pprev   = ones(1,Body_2.N) .* Body_2.F_0;
    Body_4.eta_prev  = zeros(6,Body_4.N);
    Body_4.eta_pprev = zeros(6,Body_4.N);
    Body_4.f_prev    = ones(1,Body_4.N) .* Body_4.F_0;
    Body_4.f_pprev   = ones(1,Body_4.N) .* Body_4.F_0;
    
    % Define Joints       {Screw axis here in joint frame}
    Joint_1 = Joint('Joint_1','RIGID',[0;0;0;0;0;1],Theta(1),Theta_dot(1),Theta_ddot(1),[-pi,pi],0,BASE,Body_1);
    Joint_2 = Joint('Joint_2','RIGID',[0;0;0;1;0;0],Theta(2),Theta_dot(2),Theta_ddot(2),[-pi,pi],pi/2,Body_1,Body_2);
    Joint_3 = Joint('Joint_3','RIGID',[0;0;0;0;1;0],Theta(3),Theta_dot(3),Theta_ddot(3),[-pi,pi],0,Body_2,Body_3);
    Joint_4 = Joint('Joint_4','RIGID',[0;0;0;1;0;0],Theta(4),Theta_dot(4),Theta_ddot(4),[-pi,pi],0,Body_3,Body_4);
    Joint_5 = Joint('Joint_5','RIGID',[0;0;1;0;0;0],Theta(5),Theta_dot(5),Theta_ddot(5),[-pi,pi],0,Body_4,Body_5);
    
    % Define Robot combining the Bodies and Joints {'BASE', Joint, Body, ... , Body, 'EE'}
    Robot = { BASE    , Joint_1, Body_1 };
    Robot = { Robot{:}, Joint_2, Body_2 };
    Robot = { Robot{:}, Joint_3, Body_3 };
    Robot = { Robot{:}, Joint_4, Body_4 };
    Robot = { Robot{:}, Joint_5, Body_5 };
    Joint_EE = Joint('Joint_EE','RIGID',zeros(6,1),0,0,0,0,0,Robot{end},EE);    %[Joint]    Define a 'joint' that map CoM last Body to EE
    Robot = { Robot{:}, Joint_EE, EE };
    
end