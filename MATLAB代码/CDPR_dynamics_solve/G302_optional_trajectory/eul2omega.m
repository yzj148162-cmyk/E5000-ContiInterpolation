
function omega = eul2omega(eul, d_eul)
    % 输入: eul = [phi(Roll), theta(Pitch), psi(Yaw)] (ZYX顺序)
    %       d_eul = [dphi, dtheta, dpsi]
    % 输出: omega = [wx; wy; wz] (机体坐标系下的角速度)

    phi = eul(1);
    theta = eul(2);
    % psi = eul(3); % 计算角速度其实不需要 psi (Yaw)，因为它只影响方位不影响机体相对自身的转速投影

    dphi = d_eul(1);
    dtheta = d_eul(2);
    dpsi = d_eul(3);

    % 正确的 ZYX / RPY 角速度合成公式
    omega_x = dphi - sin(theta)*dpsi;
    omega_y = cos(phi)*dtheta + sin(phi)*cos(theta)*dpsi;
    omega_z = -sin(phi)*dtheta + cos(phi)*cos(theta)*dpsi;

    omega = [omega_x; omega_y; omega_z];
end
