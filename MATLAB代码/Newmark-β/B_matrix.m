function B = B_matrix(phi, theta)
% 欧拉角速率到体角速度的变换矩阵
% omega_body = B * [phi_dot; theta_dot; psi_dot]
B = [1, 0, -sin(theta);
    0, cos(phi), sin(phi)*cos(theta);
    0, -sin(phi), cos(phi)*cos(theta)];
end