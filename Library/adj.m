function adG = adj(g)
% Lie Bracket Operator on SE(3)
% From Lie Algebra (twists) to Tensor Mapping Lie Algebra 
% (6x1) g -> (6x6) adj(g)
%
% Written by BD Bhai

    gu = g(1:3);
    gw = g(4:6);
    
    adG = [ hat(gw)  , hat(gu)  ; ...
            zeros(3) , hat(gw) ];
end