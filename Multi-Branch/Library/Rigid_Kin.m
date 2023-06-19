function [g_cur,g_act_wrt_prev,eta,d_eta] = Rigid_Kin(g_old, g_old2cur, Joint, eta_old, d_eta_old)
% Kinematic Equations to Determine Velocity of Rigid Bodies Connected with an Actuated Joint (!! RIGID ONLY !!)
% 
% DETERMINE:    Transformation, Velocities of Current Body
% GIVEN:        Transformation, Velocities of Previous Body and Connecting Joint
% 
%     - g_cur:          Transformation from Base Frame to i_th Body CoM BCF in RRC
%     - g_act_wrt_prev: Transformation from i_th Body CoM BCF to i-1_th Body CoM BCF in RAC
%     - eta:            Twist for Body velocity of i_th CoM expressed in i_th CoM BCF
%     - d_eta:          Time-Rate of Change of Body Velocity twists of i_th CoM expressed in i_th CoM BCF
%     - g_old:          Transformation from Base Frame to i-1_th Body CoM BCF in RRC
%     - g_old2cur:      Transformation from Old Body CoM to Current Body CoM
%     - Joint:          Joint Definition including Twist, Position, Velocity, Acceleration
%     - eta_old:        Twist for Body velocity of i-1_th CoM expressed in i-1_th CoM BCF
%     - d_eta_old:      Time-Rate of Change of Body Velocity twists of i-1_th CoM expressed in i-1_th CoM BCF
%     - RRC:            Robot Reference Configuration
%     - RAC:            Robot Actuated Configuration
%     - BCF:            Body Coordinate Frame
% 
% Written by BD Bhai
        
        Joint.Twist = Ad(expm3(hat(-Joint.Child.CoM))) * ...    %[Ad(SE(3)] Transform Twist from Joint Axis to i_th Body CoM
                        Joint.Twist;                            %[se(3)]    Redefine Joint Twist to now be about Child CoM
        
        g_cur = g_old * g_old2cur;                              %[SE(3)]    Current Transformation from Base Frame to i_th Body CoM BCF in RRC
        g_cur_wrt_prev = inv(g_old2cur);                        %[SE(3)]    Transformation from i_th Body CoM BCF to i-1_th Body CoM BCF in RRC
        g_act_wrt_prev = expm3(-hat(Joint.Twist)*Joint.Position) ...
            * g_cur_wrt_prev;                                   %[SE(3)]    Transformation from i_th Body CoM BCF to i-1_th Body CoM BCF in RAC
        
        eta = Ad(g_act_wrt_prev) * eta_old + ...                %[se(3)]    Twist for Body velocity of i_th CoM expressed in BCF
            Joint.Twist*Joint.Vel;                              %[se(3)]    Sum of i-1_th CoM twist in i_th CoM BCF and Joint Actuation in BCF
        d_eta = Ad(g_act_wrt_prev) * d_eta_old + adj(eta) ...   %[R3]   Time-Rate of Change of Body Velocity twists
            * Joint.Twist*Joint.Vel + Joint.Twist*Joint.Accel;  
end