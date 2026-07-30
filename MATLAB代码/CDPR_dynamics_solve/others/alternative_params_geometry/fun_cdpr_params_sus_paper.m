function para_cdpr = fun_cdpr_params_sus_paper
mass_ee = 50/9.8; %mass of the EE(kg)



b1_g = [1.992;  -0.174; 2; 1]; %(m)
b2_g = [1.992; 0.174; 2; 1];
b3_g = [ -0.845; 1.183; 2; 1];
b4_g = [ -1.147;  1.638; 2; 1];
b5_g = [-1.147;  -1.638; 2; 1];
b6_g = [-0.845; -1.813; 2; 1];
b7_g = [-2; 0; 2; 1];
b8_g = [1 ; 1.73; 2; 1];


base_g=[b1_g b2_g b3_g b4_g b5_g b6_g b7_g b8_g];




a1_e = [ 0.1075;-0.1687 ;0; 1]; %(m)
a2_e = [0.1075; 0.1687; 0; 1];
a3_e = [0.0923; 0.1774; 0; 1];
a4_e = [-0.1998; 0.0087; 0; 1];
a5_e = [-0.1998; -0.0087; 0;1];
a6_e = [0.0923; -0.1774; 0; 1];
a7_e = [ -0.1998; 0.0087; 0; 1];
a8_e = [0.1075; 0.1687; 0; 1];






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
