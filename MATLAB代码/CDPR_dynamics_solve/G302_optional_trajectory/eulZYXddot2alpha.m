% function alpha = eulZYXddot2alpha(eul, d_eul,dd_eul)
% 
% phi = eul(1);
% theta = eul(2);
% psi = eul(3);
% dphi = d_eul(1);
% dtheta = d_eul(2);
% dpsi = d_eul(3);
% ddphi = dd_eul(1);
% ddtheta = dd_eul(2);
% ddpsi = dd_eul(3);
% 
% 
% alpha = zeros(3,1);
% 
% alpha(1) = ddphi ...
%            - ddpsi*sin(theta) ...
%            - dpsi*dtheta*cos(theta);
% 
% alpha(2) = ddtheta*cos(phi) ...
%            - dtheta*dphi*sin(phi) ...
%            + ddpsi*sin(phi)*cos(theta) ...
%            + dpsi*cos(phi)*cos(theta)*dphi ...
%            - dpsi*sin(phi)*sin(theta)*dtheta;
% 
% alpha(3) = -ddtheta*sin(phi) ...
%            - dtheta*dphi*cos(phi) ...
%            + ddpsi*cos(phi)*cos(theta) ...
%            - dpsi*sin(phi)*cos(theta)*dphi ...
%            - dpsi*cos(phi)*sin(theta)*dtheta;
% end


function alpha = eulZYXddot2alpha(eul, d_eul, dd_eul)
    % 解包变量
    phi = eul(1);    theta = eul(2);
    % psi = eul(3); % 计算机体角加速度其实不需要 psi 本身
    
    dphi = d_eul(1); dtheta = d_eul(2); dpsi = d_eul(3);
    
    % 1. 雅可比矩阵 J (用于映射二阶导 dd_eul)
    % 对应公式项: J * dd_eul
    J = [1,  0,        -sin(theta);
         0,  cos(phi),  sin(phi)*cos(theta);
         0, -sin(phi),  cos(phi)*cos(theta)];
     
    % 2. 雅可比矩阵的导数 dJ (用于映射一阶导 d_eul)
    % 对应公式项: dJ * d_eul
    % 注意：这里利用链式法则，dJ 内部包含了 dphi, dtheta 等项
    dJ_dt = [0,  0,         -cos(theta)*dtheta;
             0, -sin(phi)*dphi,  cos(phi)*dphi*cos(theta) - sin(phi)*sin(theta)*dtheta;
             0, -cos(phi)*dphi, -sin(phi)*dphi*cos(theta) - cos(phi)*sin(theta)*dtheta];

    % 3. 合成角加速度 (alpha = J * dd_q + dJ * d_q)
    % 确保输入向量是列向量
    if size(d_eul, 2) > 1, d_eul = d_eul'; end
    if size(dd_eul, 2) > 1, dd_eul = dd_eul'; end
    
    alpha = J * dd_eul + dJ_dt * d_eul;
end
