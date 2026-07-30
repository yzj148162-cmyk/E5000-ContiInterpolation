function [omega_global,alpha_golbal] = body_to_global(eul, omega_body,alpha_body)
    % 输入：
    % phi, theta, psi 为 ZYX 欧拉角
    % omega_body 为机体坐标系下的角速度 [omega_x, omega_y, omega_z]
    phi = eul(1);
    theta = eul(2);
    psi = eul(3);
    
    % 旋转矩阵 R
    R = [cos(theta)*cos(psi), cos(theta)*sin(psi), -sin(theta);
         sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi), sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi), sin(phi)*cos(theta);
         cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi), cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi), cos(phi)*cos(theta)];
     
    % 将角速度从机体坐标系转换到全局坐标系
    omega_global = R' * omega_body;
    alpha_golbal = R' * alpha_body;
end
