function Bdot = B_dot_matrix(phi, theta, phi_dot, theta_dot)
% 矩阵B的时间导数
dBdphi = [0, 0, 0;
    0, -sin(phi), cos(phi)*cos(theta);
    0, -cos(phi), -sin(phi)*cos(theta)];
dBdtheta = [0, 0, -cos(theta);
    0, 0, -sin(phi)*sin(theta);
    0, 0, -cos(phi)*sin(theta)];
Bdot = phi_dot*dBdphi + theta_dot*dBdtheta;
end