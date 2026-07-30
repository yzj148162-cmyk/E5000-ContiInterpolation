function para_cdpr = fun_cdpr_params_xiaoyuan
mass_ee = 100; %mass of the EE(kg)

% Coordinates of the base anchor points in the global frame
% (global frame, the origin is the center of the bottom of the rectangle)
% schematic of global frame is showned below ↓↓↓
%{
                b1 ---------------- b4
               / |                 / |
              / b5                / b8
             /   |               /   |
            b2 ---------------- b3   |
            |    |               |   |
            b6   ·------------- b7 --·
            |   /     Zg| Yg     |  /
            |  /        |/_Xg    | /
            | /                  |/
            ·--------------------·
%}
% b1_g = [-7.2;  3.8; 5.2; 1]; %(m)
% b2_g = [-7.2; -3.8; 5.2; 1];
% b3_g = [ 7.2; -3.8; 5.2; 1];
% b4_g = [ 7.2;  3.8; 5.2; 1];
% b5_g = [-7.2;  3.8; 4.4; 1];
% b6_g = [-7.2; -3.8; 4.4; 1];
% b7_g = [ 7.2; -3.8; 4.4; 1];
% b8_g = [ 7.2;  3.8; 4.4; 1];
% base_g=[b1_g b2_g b3_g b4_g b5_g b6_g b7_g b8_g];
% 
% 
% % Coordinates of the attachment points in the local frame
% % (local frame, the origin is the center of the cube)
% % schematic of local frame is showned below ↓↓↓
% %{
% %对应对角点连接
%                 /-------a4---------- /
%                / |                a3 |
%               /  |                /  |
%              /   |              a2   |
%             ------a1-------------|   |
%             |    |    Ze| Ye     |   |
%             |    |      |/_Xe    |   |
%             |    --------------a5| ---
%             |   a6               |  /
%             |  /                 | /
%             | a7                 |/ 
%             /--------------a8-----
% %}
% lx = 1.5;
% la = lx/(cos(deg2rad(54))+sin(deg2rad(72)));
% s72 = sin(deg2rad(72));
% c72 = cos(deg2rad(72));
% %正对
% % a1_e = [ lx/2-la*s72;  la*(0.5+c72);  0.16; 1]; %(m)
% % a2_e = [ lx/2-la*s72; -la*(0.5+c72);  0.16; 1];
% % a3_e = [        lx/2;         -la/2;  0.16; 1];
% % a4_e = [        lx/2;          la/2;  0.16; 1];
% % a5_e = [       -lx/2;          la/2; -0.16; 1];
% % a6_e = [       -lx/2;         -la/2; -0.16; 1];
% % a7_e = [-lx/2+la*s72; -la*(0.5+c72); -0.16; 1];
% % a8_e = [-lx/2+la*s72;  la*(0.5+c72); -0.16; 1];
% 
% %交错
% a1_e = [ lx/2-la*s72; -la*(0.5+c72);  0.16; 1]; %(m)
% % a2_e = [ lx/2-la*s72; -la*(0.5+c72);  0.16; 1];
% a2_e = [        lx/2;         -la/2;  0.16; 1];
% a3_e = [        lx/2;          la/2;  0.16; 1];
% a4_e = [ lx/2-la*s72;  la*(0.5+c72);  0.16; 1];
% % a5_e = [       -lx/2;          la/2; -0.16; 1];
% a5_e = [-lx/2+la*s72;  la*(0.5+c72); -0.16; 1];
% a6_e = [       -lx/2;          la/2; -0.16; 1];
% a7_e = [       -lx/2;         -la/2; -0.16; 1];
% a8_e = [-lx/2+la*s72; -la*(0.5+c72); -0.16; 1];


Lx_half = 4;
Ly_half = 3.55;
% Lz = 4.8;
Lz = 3.8;
gap = 0.5;


b1_g = [(Lx_half-gap);  -Ly_half; Lz; 1]; %(m)
b2_g = [Lx_half; -(Ly_half-gap); Lz; 1];
b3_g = [ Lx_half; (Ly_half-gap); Lz; 1];
b4_g = [ (Lx_half-gap);  Ly_half; Lz; 1];
b5_g = [-(Lx_half-gap);  Ly_half; Lz; 1];
b6_g = [-Lx_half; (Ly_half-gap); Lz; 1];
b7_g = [-Lx_half; -(Ly_half-gap); Lz; 1];
b8_g = [-(Lx_half-gap) ; -Ly_half; Lz; 1];


base_g=[b1_g b2_g b3_g b4_g b5_g b6_g b7_g b8_g];


% l_ax = 0.5;
% l_ay = 0.5;
% l_az = 0.6;
l_ax = 0.25;
l_ay = 0.25;
l_az = 0.3;
a1_e = [ -l_ax;-l_ay ; -l_az; 1]; %(m)+0.05
a2_e = [l_ax; l_ay; l_az; 1];
a3_e = [l_ax; -l_ay; -l_az; 1];%+0.05
a4_e = [ -l_ax; l_ay; l_az; 1];
a5_e = [l_ax; l_ay; -l_az;1];%+0.05
a6_e = [ -l_ax; -l_ay; l_az; 1];
a7_e = [ -l_ax; l_ay; -l_az; 1];%+0.05
a8_e = [l_ax; -l_ay; l_az; 1];


% convert the origin to the center of mass by adding offset
CoM_x = 0;
CoM_y = 0;
CoM_z = -0.55*2/3;%-0.1
a1_e = a1_e - [CoM_x; CoM_y; CoM_z; 0];
a2_e = a2_e - [CoM_x; CoM_y; CoM_z; 0];
a3_e = a3_e - [CoM_x; CoM_y; CoM_z; 0];
a4_e = a4_e - [CoM_x; CoM_y; CoM_z; 0];
a5_e = a5_e - [CoM_x; CoM_y; CoM_z; 0];
a6_e = a6_e - [CoM_x; CoM_y; CoM_z; 0];
a7_e = a7_e - [CoM_x; CoM_y; CoM_z; 0];
a8_e = a8_e - [CoM_x; CoM_y; CoM_z; 0];
attach_e = [a1_e a2_e a3_e a4_e a5_e a6_e a7_e a8_e];

ao_e = [0; 0; 0; 1]; %the origin of the local frame

para_cdpr.mass_ee = mass_ee;
para_cdpr.base_g = base_g;
para_cdpr.attach_e = attach_e;
para_cdpr.ao_e = ao_e;

end