function [pose_trj, v_trj, a_trj, t_vec_G,t_end] = plan_trj_case2_line( t_step)




p_start1 = [0; 0; 5; 0; 0; 0];
p_end1 = [0; -5; 5; 0; 0;0];
% p_end1 = [0; 0; 7.5; 0; 0;0];
t_start1 = 0;  
t_end1 = 1.6

[pose_trj1, v_trj1, a_trj1, t_vec_G1] = ...
    plan_trj_avr_line(t_start1, t_step, t_end1, p_start1, p_end1);
  


pose_trj = [pose_trj1];
v_trj = [v_trj1];
a_trj = [a_trj1];
t_vec_G = [t_vec_G1];


% pose_trj = [pose_trj1, pose_trj2(:,(2:end)), pose_trj3(:,(2:end)), pose_trj4(:,(2:end))];
% v_trj = [v_trj1, v_trj2(:,(2:end)), v_trj3(:,(2:end)), v_trj4(:,(2:end))];
% a_trj = [a_trj1, a_trj2(:,(2:end)), a_trj3(:,(2:end)), a_trj4(:,(2:end))];
% t_vec_G = [t_vec_G1, t_vec_G2(:,(2:end)), t_vec_G3(:,(2:end)), t_vec_G4(:,(2:end))];

t_end = t_end1;



end