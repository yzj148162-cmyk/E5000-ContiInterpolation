function para_cdpr = fun_cdpr_params_sus_cubic
mass_ee = 1000; %mass of the EE(kg)

% Coordinates of the base anchor points in the global frame
% (global frame, the origin is the center of the bottom of the rectangle)
% schematic of global frame is showned below ↓↓↓
%{

%}


b1_g = [9;  -10; 10; 1]; %(m)
b2_g = [10; -9; 10; 1];
b3_g = [ 10; 9; 10; 1];
b4_g = [ 9;  10; 10; 1];
b5_g = [-9;  10; 10; 1];
b6_g = [-10; 9; 10; 1];
b7_g = [-10; -9; 10; 1];
b8_g = [-9 ; -10; 10; 1];


base_g=[b1_g b2_g b3_g b4_g b5_g b6_g b7_g b8_g];


% Coordinates of the attachment points in the local frame
% (local frame, the origin is the center of the cube)
% schematic of local frame is showned below ↓↓↓
%{
%对应对角点连接
                /-------a4---------- /
               / |                a3 |
              /  |                /  |
             /   |              a2   |
            ------a1-------------|   |
            |    |    Ze| Ye     |   |
            |    |      |/_Xe    |   |
            |    --------------a5| ---
            |   a6               |  /
            |  /                 | /
            | a7                 |/ 
            /--------------a8-----
%}
% lx = 1.5;
% la = lx/(cos(deg2rad(54))+sin(deg2rad(72)));
% s72 = sin(deg2rad(72));
% c72 = cos(deg2rad(72));
%正对
% a1_e = [ lx/2-la*s72;  la*(0.5+c72);  0.16; 1]; %(m)
% a2_e = [ lx/2-la*s72; -la*(0.5+c72);  0.16; 1];
% a3_e = [        lx/2;         -la/2;  0.16; 1];
% a4_e = [        lx/2;          la/2;  0.16; 1];
% a5_e = [       -lx/2;          la/2; -0.16; 1];
% a6_e = [       -lx/2;         -la/2; -0.16; 1];
% a7_e = [-lx/2+la*s72; -la*(0.5+c72); -0.16; 1];
% a8_e = [-lx/2+la*s72;  la*(0.5+c72); -0.16; 1];

%交错
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

a1_e = [ -1.25;-1.25 ; -1.5; 1]; %(m)
a2_e = [1.25; 1.25; 1.5; 1];
a3_e = [1.25; -1.25; -1.5; 1];
a4_e = [ -1.25; 1.25; 1.5; 1];
a5_e = [1.25; 1.25; -1.5;1];
a6_e = [ -1.25; -1.25; 1.5; 1];
a7_e = [ -1.25; 1.25; -1.5; 1];
a8_e = [1.25; -1.25; 1.5; 1];






% convert the origin to the center of mass by adding offset
CoM_x = 0;
CoM_y = 0;
CoM_z = 0;%-0.1
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
% para_cdpr.I_ee = [ 754 0 0;0 754 0;0 0  547];
end
