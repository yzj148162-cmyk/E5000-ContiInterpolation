function [pose_trj, v_trj, a_trj, t_vec_G,t_end] = plan_trj_xiaoyuan( t_step)




p_start1 = [-1/1.414*3.5; 0; 1; 0; -pi/4; 0];
% p_end1 = [0; 0; 0.7; 0.7; 0.192; -0.192];
p_end1 = [1/1.414*3.5; 0; 1; 0; pi/4; 0];
t_start1 = 0;  
t_end1 = 10;
R = 1*1.414/1.414*3.5;
vec = [0 -1 0];
[pose_trj1, v_trj1, a_trj1, t_vec_G1] = ...
    plan_trj_curve_xiaoyuan(t_start1, t_step, t_end1, p_start1, p_end1, R,vec);
  







pose_trj = [pose_trj1];
v_trj = [v_trj1];
a_trj = [a_trj1];
t_vec_G = [t_vec_G1];

t_end = t_end1;



end