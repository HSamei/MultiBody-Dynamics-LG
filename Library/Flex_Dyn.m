function [g_end,f,eta,d_eta_end] = Flex_Dyn(g_base, F_dist, F_base, BODY, eta_base, c0, c1, c2)
% Integrate the Cosserat PDE for a Flexible Body to return the states and transformation at its end.
% 
% DETERMINE:    Configuration, Acceleration @ End & Velocity, Strain over Body
% GIVEN:        Transformation, Velocity, Wrench BC @ Base, FDM Coeff, Body Definition, Applied Loads
% 
%     - g_base:     Transformation to the Base Frame of the Body wrt to Reference Frame
%     - F_dist:     Applied Distributed Wrench on the Body (expressed in Body Frame)
%     - F_base:     Twist for Body velocity of i_th CoM expressed in i_th CoM BCF
%     - BODY:       Body Object with Relevant Informaiton
%     - eta_base:   Velocity Twist of Base Frame
%     - c0:         FDM Coeff - Current Time Step
%     - c1:         FDM Coeff - Previous Time Step
%     - c2:         FDM Coeff - PrePrevious Time Step
%     - d_eta:      Body Acceleration Twists
%     - eta(_h):    Body Velocity Twists (Finite Difference Approximation using previous values)
%     - f(_h):      Body Strain Twists   (Finite Difference Approximation using previous values)
%     - eta_s:      Velocity Spatial Rate Twists
%     - f_s:        Strain Spatial Rate Twists
%     - f_sh:       Strain Spatial Rate Twist (Finite Difference Approximation using previous values)
% 
% Written by BD Bhai
    
    %% Allocate Memory
    eta = zeros(6,BODY.N);      %[se(3)]    Body Velocity Twist at Local Cross-Section
    f   = zeros(6,BODY.N);      %[se(3)]    Body Strain Twist at Local Cross-Section
    g   = zeros(4,4,BODY.N);    %[SE(3)]    Transformation to Local Cross-Section
    
    eta_h = c1*BODY.eta_prev + c2*BODY.eta_pprev;   %[se(3)]    Body Velocity Twist History Terms in Local Coordinates
    f_h = c1*BODY.f_prev + c2*BODY.f_pprev;         %[se(3)]    Body Strain Twist History Terms in Local Coordinates
    
    %% Initialize States
    f(:,1) = BODY.Stiff \ F_base + BODY.F_0;        %[se(3)]    Assign Initial Strain Twist @ Base of Body
    eta(:,1) = eta_base;                            %[se(3)]    Assign Initial Velocity Twist @ Base of Body
    g(:,:,1) = g_base;                              %[SE(3)]    Assign Configuration @ Base of Body wrt to Reference Frame
    ds = BODY.L / (BODY.N - 1);                     %[m]        Spatial Step Size assumed in Numerical Integration
    
    %% Integrate Coss ODE Numerically
    for i = 1 : BODY.N - 1
        f_sh = ( c1*(BODY.f_prev(:,i+1) - BODY.f_prev(:,i)) + ...
                 c2*(BODY.f_pprev(:,i+1) - BODY.f_pprev(:,i)) ) / ds;
             
        [f_s,eta_s] = Coss_ODE(eta(:,i), f(:,i), eta_h(:,i), f_h(:,i), f_sh, ...
            BODY.Stiff,BODY.Damp,BODY.Mass,c0,BODY.F_0,F_dist(:,i));
        
        f(:,i+1) = f(:,i) + f_s*ds;                     %[se(3)]    Assuming Euler Integration
        eta(:,i+1) = eta(:,i) + eta_s*ds;               %[se(3)]    Assuming Euler Integration
        g(:,:,i+1) = g(:,:,i) * expm(hat(f(:,i))*ds);   %[SE(3)]    Assuming Lie-Euler Geometric Integration
    end
    
    g_end = g(:,:,end);
    d_eta_end = c0*eta(:,end) + eta_h(:,end);       %[]     Report the body acceleration at end of Body using the adopted FDM
end