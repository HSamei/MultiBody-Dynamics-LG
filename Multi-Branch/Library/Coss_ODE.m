function [f_s,eta_s] = Coss_ODE(eta,f,eta_h,f_h,f_sh,K,C,M,c0,f_0,Fd_ext)
% Cosserat Model for a Continuum Semi-Discretized from PDE into a Spatial ODE (!! FLEXIBLE ONLY !!)
% 
% DETERMINE:    Spatial Derivative of Velocity and Strain Twists
% GIVEN:        Velocity, Strain Twists & Stiffness & Mass & Free Strain & Applied Loading
% 
%     - eta(_h):    Velocity Twists (Finite Difference Approximation using previous values)
%     - f(_h):      Strain Twists   (Finite Difference Approximation using previous values)
%     - eta_s:      Velocity Spatial Rate Twists
%     - f_s:        Strain Spatial Rate Twists
%     - f_sh:       Strain Spatial Rate Twist (Finite Difference Approximation using previous values)
%     - f_0:        Free Strain of Body under no Applied Loads
%     - c_0:        FDM Coefficient for Time Discretization
%     - K:          Stiffness Matrix of Body in Local BCF
%     - M:          Mass Matrix of Body in Local BCF
%     - Fd_ext:     Applied Distributed Load over body expressed in Local BCF
%
% Written by BD Bhai


       % Time Discretization (definition of c0 & X_h based on FDM used)
       f_t = c0*f + f_h;                   % Local Time Discretization for history in Local Coordinates
       eta_t = c0*eta + eta_h;             % Local Time Discretization for history in Local Coordinates
       
       % Spatial Derivatives
       f_s = (K + C*c0) \ (M*eta_t - (adj(eta)')*M*eta - C*f_sh + (adj(f)')*(K*(f - f_0) + C*f_t) + Fd_ext);
       eta_s = f_t + adj(eta)*f;
end