function q = inverseKinematics(H, a, alpha, d, theta_offset)

    % DH parameters
    a1 = a(1);
    a2 = a(2);
    a3 = a(3);
    d1 = d(1);
    d4 = d(4);
    d6 = d(6);
    a6 = a(6);

    rd = H(1:3, 1:3);

    % Wrist center
    vector_c = rd * [a6; 0; d6];
    xc = H(1,4) - vector_c(1);
    yc = H(2,4) - vector_c(2);
    zc = H(3,4) - vector_c(3);

    r = sqrt(xc^2 + yc^2) - a1;
    l = sqrt(a3^2 + d4^2);
    s = zc - d1;
    h = sqrt(r^2 + s^2);

    D = (h^2 - a2^2 - l^2) / (2*a2*l);
    gamma = atan2(a3, d4);
    phi = atan2(sqrt(1 - D^2), D);

    q = zeros(6,1);

    % First three joints
    q(1) = atan2(yc, xc);
    q(2) = atan2(s, r) + atan2(l*sin(phi), a2 + l*cos(phi));
    q(3) = pi/2 - phi - gamma;

    % Orientation
    R_3 = eye(3);
    for i = 1:3
        th = q(i) + theta_offset(i);
        A = [ cos(th) -sin(th)*cos(alpha(i))  sin(th)*sin(alpha(i))  a(i)*cos(th);
              sin(th)  cos(th)*cos(alpha(i)) -cos(th)*sin(alpha(i))  a(i)*sin(th);
              0        sin(alpha(i))           cos(alpha(i))          d(i);
              0        0                       0                      1 ];
        R_3 = R_3 * A(1:3,1:3);
    end

    R36 = R_3' * rd;

    q(4) = atan2(R36(2,3), R36(1,3));
    q(5) = atan2(sqrt(R36(1,3)^2 + R36(2,3)^2), R36(3,3));
    q(6) = atan2(R36(3,2), -R36(3,1));
end
