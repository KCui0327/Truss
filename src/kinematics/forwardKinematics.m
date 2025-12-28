function H = forwardKinematics(joint, a, alpha, d, theta_offset)
% ---------------------------------------------------------
% forward_kuka.m
% Computes FK using standard DH parameters
%
% INPUTS:
%   joint        - Nx1 joint angles (rad)
%   a            - Nx1 DH a parameters
%   alpha        - Nx1 DH alpha parameters
%   d            - Nx1 DH d parameters
%   theta_offset - Nx1 joint angle offsets
%
% OUTPUT:
%   H - 4x4 homogeneous transformation matrix
% ---------------------------------------------------------

    n = length(joint);
    H = eye(4);

    for i = 1:n
        th = joint(i) + theta_offset(i);

        hi = [ cos(th)  -sin(th)*cos(alpha(i))   sin(th)*sin(alpha(i))   a(i)*cos(th);
               sin(th)   cos(th)*cos(alpha(i))  -cos(th)*sin(alpha(i))   a(i)*sin(th);
               0         sin(alpha(i))           cos(alpha(i))           d(i);
               0         0                        0                       1 ];

        H = H * hi;
    end
end
