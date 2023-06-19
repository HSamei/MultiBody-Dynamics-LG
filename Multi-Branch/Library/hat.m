function xhat = hat(x)
% Lie Bracket Operator on SE(3)
% From Vector to Lie Algebra
% (3x1) || (6x1) x -> (3x3) || (4x4) hat(x)
%
% Written by BD Bhai

    if length(x) == 3                   %[]  For so(3)
        xhat = [ 0    -x(3)  x(2)  ;...
                 x(3)  0    -x(1)  ;...
                -x(2)  x(1)  0    ];
    elseif length(x) == 6               %[]  For se(3)
        xhat = [ hat(x(4:6)) , reshape(x(1:3),3,1)  ;...
                                        zeros(1,4) ];
    else
        error('Why would you need a hat when you dont have a brain?')
    end
end