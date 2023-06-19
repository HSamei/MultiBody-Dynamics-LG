function x = unhat(xhat)
% Lie Bracket Operator on SE(3)
% From Lie Algebra to Vector
% (3x3) || (4x4) hat(x) -> (3x1) || (6x1) x
%
% Written by BD Bhai

    if length(xhat(:,1)) == 3
        x = [xhat(3,2);xhat(1,3);xhat(2,1)];
    elseif length(xhat(:,1)) == 4
        x = [xhat(1,4);xhat(2,4);xhat(3,4);...
             xhat(3,2);xhat(1,3);xhat(2,1)];
    else
        x = zeros(length(xhat(:,1)));
    end
end