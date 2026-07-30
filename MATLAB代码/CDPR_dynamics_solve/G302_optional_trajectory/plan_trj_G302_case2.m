function [pose_trj, v_trj, a_trj, t_vec_G,t_end] = plan_trj_G302_case2( t_step)


p_start1 = [-0.7; -0.3; 0.8; 0; 0; 0];
p_end1 = [0.5; 0.5; 1.2; 0; 0; 0];

t_start1 = 0;  
t_end1 = 20;

O_position = [0.7;-0.3;1.2];

[pose_trj1, v_trj1, a_trj1, t_vec_G1] = ...
    plan_trj_oval_G302(t_start1, t_step, t_end1, p_start1, p_end1,O_position);
  
p_start2 = pose_trj1(:,end);
p_end2 = [0.7; 0.3; 1.2; pose_trj1(4:6,end)];
t_start2 = t_end1;  
t_end2 = 25;
R = 0.2;
vec = [0 0 -1];
[pose_trj2, v_trj2, a_trj2, t_vec_G2] = ...
    plan_trj_curve2_xiaoyuan(t_start2, t_step, t_end2, p_start2, p_end2, R,vec,O_position);
  
% [pose_trj2, v_trj2, a_trj2, t_vec_G2] = ...
%     plan_trj_oval_xiaoyuan(t_start2, t_step, t_end2, p_start2, p_end2,O_position);


% p_start2 = p_end1;
% p_end2 = [0; -2*cos(65/180*pi); 1.1+2*sin(65/180*pi); -5/180*pi; 0; 0];
% t_start2 = t_end1;  
% t_end2 = 8;
% [pose_trj2, v_trj2, a_trj2, t_vec_G2] = ...
%     plan_trj_line(t_start2, t_step, t_end2, p_start2, p_end2);
% 
% p_start3 = p_end2;
% p_end3 = [0; 0; 3.1; 25/180*pi; 0; 0];
% t_start3 = t_end2;  
% t_end3 = 12;
% R = 2;
% vec = [1 0 0];
% [pose_trj3, v_trj3, a_trj3, t_vec_G3] = ...
%     plan_trj_curve_xiaoyuan(t_start3, t_step, t_end3, p_start3, p_end3, R,vec);
% 
% 
% p_start4 = p_end3;
% p_end4 = [0; 0; 3.1; -25/180*pi; 0; 0];
% t_start4 = t_end3;  
% t_end4 = 15;
% [pose_trj4, v_trj4, a_trj4, t_vec_G4] = ...
%     plan_trj_line(t_start4, t_step, t_end4, p_start4, p_end4);


% pose_trj = [pose_trj1, pose_trj2(:,(2:end)), pose_trj3(:,(2:end)), pose_trj4(:,(2:end))];
% v_trj = [v_trj1, v_trj2(:,(2:end)), v_trj3(:,(2:end)), v_trj4(:,(2:end))];
% a_trj = [a_trj1, a_trj2(:,(2:end)), a_trj3(:,(2:end)), a_trj4(:,(2:end))];
% t_vec_G = [t_vec_G1, t_vec_G2(:,(2:end)), t_vec_G3(:,(2:end)), t_vec_G4(:,(2:end))];
pose_trj = [pose_trj1, pose_trj2(:,(2:end))];
v_trj = [v_trj1, v_trj2(:,(2:end))];
a_trj = [a_trj1, a_trj2(:,(2:end))];
t_vec_G = [t_vec_G1, t_vec_G2(:,(2:end))];
t_end = t_end2;

% pose_trj = [pose_trj1];
% v_trj = [v_trj1];
% a_trj = [a_trj1];
% t_vec_G = [t_vec_G1];
% t_end = t_end1;



end