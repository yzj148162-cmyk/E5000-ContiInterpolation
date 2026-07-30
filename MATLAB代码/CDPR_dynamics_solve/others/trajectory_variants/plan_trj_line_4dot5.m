function [pose_trj, v_trj, a_trj, t_vec_G,t_end] = plan_trj_line_4dot5( t_step)



% p_start1 = [0.72;0;0; 10.9942/180*pi;-17.1784/180*pi;3.1473/180*pi];

% p_start1 = [0; 0; 0; 0; 0; 0];
p_start1 = [0; 0; 0.7; -0.29; 0; 0];
% p_end1 = [0; 0; 0.7; 0.7; 0.192; -0.192];
p_end1 = [0; 0; 0.7; 0.6; 0; 0];
t_start1 = 0;  
t_end1 = 4;

[pose_trj1, v_trj1, a_trj1, t_vec_G1] = ...
    plan_trj_line(t_start1, t_step, t_end1, p_start1, p_end1);
  



p_start2 = p_end1;
p_end2 = [0; 0; 0.7; 0.4; 0.192; -0.192];
t_start2 = t_end1;
t_end2 = 5;
[pose_trj2, v_trj2, a_trj2, t_vec_G2] = ...
    plan_trj_line(t_start2, t_step, t_end2, p_start2, p_end2);
% 


% p_start3 = p_end2;
% p_end3 = [0; 0; 0.7; 0.3; -0.3; 0.182];
% t_start3 = t_end2;
% t_end3 = 9;
% [pose_trj3, v_trj3, a_trj3, t_vec_G3] = ...
%     plan_trj_line(t_start3, t_step, t_end3, p_start3, p_end3);



p_start3 = p_end2;
p_end3 = [0; 0; 0.72; 0.2; -0.3; 0.192];
t_start3 = t_end2;
t_end3 = 7.5;
[pose_trj3, v_trj3, a_trj3, t_vec_G3] = ...
    plan_trj_line(t_start3, t_step, t_end3, p_start3, p_end3);
% 


p_start4 = p_end3;
p_end4 = [0; 0; 0.7; -0.9; 0; 0];
t_start4 = t_end3;
t_end4 = 10;

% 
[pose_trj4, v_trj4, a_trj4, t_vec_G4] = ...
    plan_trj_line(t_start4, t_step, t_end4, p_start4, p_end4);


p_start5 = p_end4;
p_end5 = [0; 0; 0.7; 0; 0; 0];
t_start5 = t_end4;
t_end5 = 14;
[pose_trj5, v_trj5, a_trj5, t_vec_G5] = ...
    plan_trj_line(t_start5, t_step, t_end5, p_start5, p_end5);


p_start6 = p_end5;
p_end6 = [1.1; 0; 0.7; 0; 0; 0];
t_start6 = t_end5;
t_end6 = 15;
[pose_trj6, v_trj6, a_trj6, t_vec_G6] = ...
    plan_trj_line(t_start6, t_step, t_end6, p_start6, p_end6);

% p_start5 = p_end4;
% p_end5 = [1.1; 0; 0.7; 0; 0; 0];
% t_start5 = t_end4;
% t_end5 = 10;
% [pose_trj5, v_trj5, a_trj5, t_vec_G5] = ...
%     plan_trj_line(t_start5, t_step, t_end5, p_start5, p_end5);
p_start7 = p_end6;
p_end7 = [0; -1.15; 0.7; 0; 0; 0];
t_start7 = t_end6;
t_end7 = 17;
[pose_trj7, v_trj7, a_trj7, t_vec_G7] = ...
    plan_trj_line(t_start7, t_step, t_end7, p_start7, p_end7);

% p_start6 = p_end5;
% p_end6 = [0; -1.15; 0.7; 0; 0; 0];
% t_start6 = t_end5;
% t_end6 = 12;
% [pose_trj6, v_trj6, a_trj6, t_vec_G6] = ...
%     plan_trj_line(t_start6, t_step, t_end6, p_start6, p_end6);

p_start8 = p_end7;
p_end8 = [0; -0.2; 1.5; 0; 0; 0];
t_start8 = t_end7;
t_end8 = 18;
[pose_trj8, v_trj8, a_trj8, t_vec_G8] = ...
    plan_trj_line(t_start8, t_step, t_end8, p_start8, p_end8);

% p_start7 = p_end6;
% p_end7 = [0; -0.2; 1.5; 0; 0; 0];
% t_start7 = t_end6;
% t_end7 = 13;
% [pose_trj7, v_trj7, a_trj7, t_vec_G7] = ...
%     plan_trj_line(t_start7, t_step, t_end7, p_start7, p_end7);


p_start9 = p_end8;
p_end9 = [0;2; 3; 0; 0; 0];
t_start9 = t_end8;
t_end9 = 22;
[pose_trj9, v_trj9, a_trj9, t_vec_G9] = ...
    plan_trj_line(t_start9, t_step, t_end9, p_start9, p_end9);

% p_start8 = p_end7;
% p_end8 = [0;2; 3; 0; 0; 0];
% t_start8 = t_end7;
% t_end8 = 17;
% [pose_trj8, v_trj8, a_trj8, t_vec_G8] = ...
%     plan_trj_line(t_start8, t_step, t_end8, p_start8, p_end8);


% [-1.5;0; 2; 0; 0; 0]

p_start10 = p_end9;
p_end10 = p_start1;
t_start10 = t_end9;
t_end10 = 25;

[pose_trj10, v_trj10, a_trj10, t_vec_G10] = ...
    plan_trj_line(t_start10, t_step, t_end10, p_start10, p_end10);



% pose_trj = [pose_trj1, pose_trj2(:,(2:end)), pose_trj3(:,(2:end)), pose_trj4(:,(2:end)), pose_trj5(:,(2:end)), pose_trj6(:,(2:end)), pose_trj7(:,(2:end)), pose_trj8(:,(2:end)), pose_trj9(:,(2:end))];
% v_trj = [v_trj1, v_trj2(:,(2:end)), v_trj3(:,(2:end)), v_trj4(:,(2:end)), v_trj5(:,(2:end)), v_trj6(:,(2:end)), v_trj7(:,(2:end)), v_trj8(:,(2:end)), v_trj9(:,(2:end))];
% a_trj = [a_trj1, a_trj2(:,(2:end)), a_trj3(:,(2:end)), a_trj4(:,(2:end)), a_trj5(:,(2:end)), a_trj6(:,(2:end)), a_trj7(:,(2:end)), a_trj8(:,(2:end)), a_trj9(:,(2:end))];
% t_vec_G = [t_vec_G1, t_vec_G2(:,(2:end)), t_vec_G3(:,(2:end)), t_vec_G4(:,(2:end)), t_vec_G5(:,(2:end)), t_vec_G6(:,(2:end)), t_vec_G7(:,(2:end)), t_vec_G8(:,(2:end)), t_vec_G9(:,(2:end))];
% 
% t_end = t_end9;

pose_trj = [pose_trj1, pose_trj2(:,(2:end)), pose_trj3(:,(2:end)), pose_trj4(:,(2:end)), pose_trj5(:,(2:end)), pose_trj6(:,(2:end)), pose_trj7(:,(2:end)), pose_trj8(:,(2:end)), pose_trj9(:,(2:end)),pose_trj10(:,(2:end))];
v_trj = [v_trj1, v_trj2(:,(2:end)), v_trj3(:,(2:end)), v_trj4(:,(2:end)), v_trj5(:,(2:end)), v_trj6(:,(2:end)), v_trj7(:,(2:end)), v_trj8(:,(2:end)), v_trj9(:,(2:end)), v_trj10(:,(2:end))];
a_trj = [a_trj1, a_trj2(:,(2:end)), a_trj3(:,(2:end)), a_trj4(:,(2:end)), a_trj5(:,(2:end)), a_trj6(:,(2:end)), a_trj7(:,(2:end)), a_trj8(:,(2:end)), a_trj9(:,(2:end)), a_trj10(:,(2:end))];
t_vec_G = [t_vec_G1, t_vec_G2(:,(2:end)), t_vec_G3(:,(2:end)), t_vec_G4(:,(2:end)), t_vec_G5(:,(2:end)), t_vec_G6(:,(2:end)), t_vec_G7(:,(2:end)), t_vec_G8(:,(2:end)), t_vec_G9(:,(2:end)), t_vec_G10(:,(2:end))];

t_end = t_end10;




end