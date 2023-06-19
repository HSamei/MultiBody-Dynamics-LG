function [BC_START,BC_END,NumBod] = MB_Flex_ID(varargin)
% Identify the number of bodies and the flexible bodies in the manipulator.
% 
% DETERMINE:    First Flexible Body, Last Flexible Body, Number of Bodies
% GIVEN:        Manipulator Definition & Joint Position, Velocity, Acceleration
% 
%     - ROBOT:      Definition of Manipulator (Collection of bodies and joints)
%     - BC_START:   First Flexible Body in Manipulator
%     - BC_END:     Last Flexible Body in Manipulator
%     - NumBod:     Number of Bodies in the Manipulator
% 
% Written by BD Bhai

    NumBod = (length(varargin{1}) - 3) / 2; %[]     Number of Bodies in the Manipulator
    
    BC_START = NumBod + 3;                  %[]     Initialize as 1 Beyond Last Body
    BC_END = 0;                             %[]     Initialize as 1 Before First Body
    for i = 1 : NumBod + 2                  %[]     Iterate Bodies from Base to EE (N+2)
       if strcmp(varargin{1}{2*i-1}.Type,'FLEXIBLE')
           if BC_START > i
                BC_START = i;
           end
           if BC_END < i
               BC_END = i;
           end
       end
    end
    
    if length(varargin{1}) <= 4             %[]     Check Min Number of Components for Manipulator
        error('You Unstructured Blob, a LINKLESS/JOINTLESS manipulator?')
    end
    
    for i = 2:length(varargin)              %[]     Check Joints and Corresponding Inputs Consistency
        if length(varargin{i}) ~= NumBod 
            error('You Egg, Inconsistent Model and Joint Actuation')
        end
    end
end