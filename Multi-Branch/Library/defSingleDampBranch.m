function Robot = defSingleDampBranch(Theta,Theta_dot,Theta_ddot)
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
    r = .01;             %[m]        Radius of Cross-Section
    E = .6e6;            %[Pa]       Youngs Modulus
    G = E/(2*(1+.3));    %[Pa]       Shear Modulus
    A = pi*r^2;          %[m2]       Cross-Sectional Area of Beam
    I = pi/4*r^4;        %[m4]       2nd Moment of Inertia of Beam
    N = 21;              %[]         Number of Discretized Elements
    rho = 75e1;          %[kg/m3]    Density of Material
    mu  = 1.2e4;         %[N/m^2s]   Viscosity of Peanut Butter
%     mu  = 0;           %[N/m^2s]   Viscosity of Nothing
    
    J = diag([2*I,I,I]);            %[m4]       3D Moment of Inertia of Beam
    Kbt = diag([2*G*I,E*I,E*I]);    %[Nm2]      Bending and Torsional Rigidity (Rotational)
    Kse = diag([E*A,G*A,G*A]);      %[N]        Shear and Extension Rigidity (Linear)
    Cse = diag([3*A,A,A])*mu;       %[N/s]      Shear and Extension Damping (Linear)
    Cbt = diag([2*I,I,I])*mu;       %[N/s]      Bending and Torsional Damping (Rotational)
    
    K = [Kse,zeros(3);zeros(3),Kbt];
    C = [Cse,zeros(3);zeros(3),Cbt];
    Mf = [rho*A*eye(3),zeros(3);zeros(3),rho*J];
    Z = zeros(6,1);
    
    %% DEFINE THE MANIPULATOR
    % Define Bodies
    BASE   = Body('BASE'  ,'RIGID',Inf*eye(6),Z,Z);
    Body_1 = Body('Body_1','FLEXIBLE',Mf,K,C,F_0,N,L_0);
    EE     = Body('EE'  ,'RIGID',zeros(6),Z,Z);
    
    % Give Previous States for Flexible Bodies
    Body_1.eta_prev  = zeros(6,Body_1.N);
    Body_1.eta_pprev = zeros(6,Body_1.N);
    Body_1.f_prev    = F_0.*ones(1,Body_1.N);
    Body_1.f_pprev   = F_0.*ones(1,Body_1.N);
    
    % Define Joints       {Screw axis here in joint frame}
    Joint_1 = Joint('Joint_1','RIGID',[0;0;0;1;0;0],Theta(1),Theta_dot(1),Theta_ddot(1),[-pi,pi],0,BASE,Body_1);
    
    % Define Robot combining the Bodies and Joints {'BASE', Joint, Body, ... , Joint, Body, 'EE'}
    Robot = { BASE    , Joint_1, Body_1 };
    Joint_EE = Joint('Joint_EE','RIGID',zeros(6,1),0,0,0,0,0,Robot{end},EE);    %[Joint]    Define a 'joint' that map CoM last Body to EE
    Robot = { Robot{:}, Joint_EE, EE };       %[]    Flexible Branch_2
    
end