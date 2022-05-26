function expmG = expm3(G)
% Matrix Exponential on se(3) or so(3)
% Mapping from Lie Algebra to Lie Group 
% (4x4) se(3) -> (4x4) SE(3)
% (3x3) s0(3) -> (3x3) SO(3)
%
% Written by BD Bhai

    if (length(G(1,:)) == 4)        %[]         For G in se(3)
        Gw = G(1:3,1:3);            %[so(3)]    Skew Symm. Element of so(3)
        Gu = G(1:3,4);              %[R(3)]     Vector Linear Component of R(3)
        magGw = (norm(unhat(Gw)));  %[]         Magnitude of Angular Component
        if magGw == 0               %[]         Check for Pure Translation
            A = eye(3);
        else
            A = eye(3) + Gw^2./magGw^3*(magGw-sin(magGw)) + Gw./magGw^2*(1-cos(magGw));
        end
        expmG = [ expm3(Gw) , A*Gu  ;...
                  zeros(1,3),    1 ];
              
    elseif (length(G(1,:)) == 3)    %[]         For G in so(3)
        Gw = G;                     %[so(3)]    Skew Symm. Element of so(3)
        magGw = (norm(unhat(Gw)));  %[]         Magnitude of Angular Component
        if magGw == 0               %[]         Check for No Rotation
            expmG = eye(3);
        else
            expmG = eye(3) + Gw./magGw*sin(magGw) + Gw^2./magGw^2*(1-cos(magGw));
        end
        
    else                            %[]         For Invalid Input G
        error('You are going to need exponentially more help ...')
    end
end