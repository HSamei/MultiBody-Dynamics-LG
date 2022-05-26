function AdG = Ad(G)
% Lie Group Adjoint Operator on SE(3)
% From Transformation to Tensor Mapping Lie Algebra to New Coordinate Frame 
% (4x4) G -> (6x6) Ad(G)
%
% Written by BD Bhai

    R = G(1:3,1:3);
    p = G(1:3,4);
    
    AdG = [ R        , hat(p)*R  ; ...
            zeros(3) , R        ];
end