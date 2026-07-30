function [pose_trj, v_trj, a_trj, t_vec_G,t_end] = plan_trj_line_test( t_step)




p_start1 = [0; 0; 3.5; 0; 0; 0];
p_end1 = [0; 0; 3.5; 0; 0; 0];
t_start1 = 0;  
t_end1 = 3;

[pose_trj1, v_trj1, a_trj1, t_vec_G1] = ...
    plan_trj_line(t_start1, t_step, t_end1, p_start1, p_end1);
  
pose_trj = pose_trj1;
v_trj = v_trj1;
a_trj = a_trj1;
t_vec_G = t_vec_G1;
t_end = t_end1;





end