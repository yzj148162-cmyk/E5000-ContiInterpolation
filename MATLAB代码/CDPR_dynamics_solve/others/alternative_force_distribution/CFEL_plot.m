clc; clear;
%% --------------------- Parameters of CDPR ---------------------------- %%

%----出绳点位置坐标

para_cdpr = fun_cdpr_params_sus_paper;

mass_ee = para_cdpr.mass_ee;
base_g = para_cdpr.base_g;
attach_e = para_cdpr.attach_e;
ao_e = para_cdpr.ao_e;
% Iee = [ 754 0 0;0 754 0;0 0  547]/2/100;

b1_g = base_g(:,1);b2_g = base_g(:,2);b3_g = base_g(:,3);b4_g = base_g(:,4);
b5_g = base_g(:,5);b6_g = base_g(:,6);b7_g = base_g(:,7);b8_g = base_g(:,8);

a1_e = attach_e(:,1);a2_e = attach_e(:,2);a3_e = attach_e(:,3);a4_e = attach_e(:,4);
a5_e = attach_e(:,5);a6_e = attach_e(:,6);a7_e = attach_e(:,7);a8_e = attach_e(:,8);

Ixx_ee = 1000; %inertial of the EE(kg*m2)
Iyy_ee = 500;
Izz_ee = 500;
M0_ee = diag([mass_ee, mass_ee, mass_ee, Ixx_ee, Iyy_ee, Izz_ee]);
Iee = diag([Ixx_ee, Iyy_ee, Izz_ee]);


search_xmin = -1;   %positions(m)
search_xmax = 1;
search_ymin = -1;
search_ymax = 1;
search_zmin = 0;
search_zmax = 0;
% 
% search_xmin = -7;   %positions(m)
% search_xmax = 7;
% search_ymin = -7;
% search_ymax = 7;
% search_zmin = 2;
% search_zmax = 8;
dot_num = 500000;     %random points

step_num = dot_num; %number of step points

rng("shuffle");
x_vals = search_xmin + (search_xmax-search_xmin)*rand(1,dot_num);
y_vals = search_ymin + (search_ymax-search_ymin)*rand(1,dot_num);
z_vals = search_zmin + (search_zmax-search_zmin)*rand(1,dot_num);

% Pose of step points in workspace
% [X, Y, Z, Ax, Ay, Az] = ndgrid(x_vals, y_vals, z_vals, ax_vals, ay_vals, az_vals);
%pose_step_points = [X(:), Y(:), Z(:), Ax(:), Ay(:), Az(;)]';
pose_step_points = [x_vals; y_vals; z_vals; zeros(1,dot_num); zeros(1,dot_num); zeros(1,dot_num)];


force_min = 1;
force_max = 100; 


num_vertex = zeros(1,step_num);
ideal_workspace = zeros(4,step_num);
ideal_cable_length = zeros(8,step_num);
ideal_cable_force = zeros(8,step_num);
ideal_a_g = zeros(48,step_num);

i_num = 0;
i_ideal = 0;


a_down_e = [0;0;-0.3-(-0.55*2/3);1];
a_up_e = [0;0;0.3-(-0.55*2/3);1];

% segmentsTrajectory = zeros(8,6,step_num);
for i_step = 1:step_num
    pose_ee = pose_step_points(:,i_step);
    x_step = pose_ee(1);
    y_step = pose_ee(2);
    z_step = pose_ee(3);
    ax_step = pose_ee(4);
    ay_step = pose_ee(5);
    az_step = pose_ee(6);


 

    % Calculate homogeneous transformation matrix
    Tr_matrix = [1, 0, 0, x_step;...
                 0, 1, 0, y_step;...
                 0, 0, 1, z_step;...
                 0, 0, 0,       1] * ...
                [cos(az_step), -sin(az_step), 0, 0;...
                 sin(az_step),  cos(az_step), 0, 0;...
                            0,             0, 1, 0;...
                            0,             0, 0, 1] * ...
                [ cos(ay_step), 0, sin(ay_step), 0;...
                             0, 1,            0, 0;...
                 -sin(ay_step), 0, cos(ay_step), 0;...
                             0, 0,            0, 1] * ...
                [1,            0,             0, 0;...
                 0, cos(ax_step), -sin(ax_step), 0;...
                 0, sin(ax_step),  cos(ax_step), 0;...
                 0,            0,             0, 1];

    % Coordinates of attachment points converted to the global frame
    a1_g = Tr_matrix * a1_e;
    a2_g = Tr_matrix * a2_e;
    a3_g = Tr_matrix * a3_e;
    a4_g = Tr_matrix * a4_e;
    a5_g = Tr_matrix * a5_e;
    a6_g = Tr_matrix * a6_e;
    a7_g = Tr_matrix * a7_e;
    a8_g = Tr_matrix * a8_e;

    a_down_g = Tr_matrix * a_down_e;
    a_up_g = Tr_matrix * a_up_e;


    
    ao_g = Tr_matrix * ao_e;
    % ao_z_g = Tr_matrix * ao_z_e;
    % ao_x_g = Tr_matrix * ao_x_e;
    % ao_y_g = Tr_matrix * ao_y_e;
    % a_smart_mess_g = Tr_matrix * a_smart_mess_e;

    % Vectors of EE in global frame (from the center to attachment points)
    vector_ee1 = a1_g - ao_g; vector_ee1(4) = [];
    vector_ee2 = a2_g - ao_g; vector_ee2(4) = [];
    vector_ee3 = a3_g - ao_g; vector_ee3(4) = [];
    vector_ee4 = a4_g - ao_g; vector_ee4(4) = [];
    vector_ee5 = a5_g - ao_g; vector_ee5(4) = [];
    vector_ee6 = a6_g - ao_g; vector_ee6(4) = [];
    vector_ee7 = a7_g - ao_g; vector_ee7(4) = [];
    vector_ee8 = a8_g - ao_g; vector_ee8(4) = [];

    % Cable vectors of corresponding attachment points (from EE to the base)
    vector_cable1 = b1_g - a1_g; vector_cable1(4) = [];
    vector_cable2 = b2_g - a2_g; vector_cable2(4) = [];
    vector_cable3 = b3_g - a3_g; vector_cable3(4) = [];
    vector_cable4 = b4_g - a4_g; vector_cable4(4) = [];
    vector_cable5 = b5_g - a5_g; vector_cable5(4) = [];
    vector_cable6 = b6_g - a6_g; vector_cable6(4) = [];
    vector_cable7 = b7_g - a7_g; vector_cable7(4) = [];
    vector_cable8 = b8_g - a8_g; vector_cable8(4) = [];

 


  





    % Ideal cable lengths
    ideal_cl1 = norm(vector_cable1);
    ideal_cl2 = norm(vector_cable2);
    ideal_cl3 = norm(vector_cable3);
    ideal_cl4 = norm(vector_cable4);
    ideal_cl5 = norm(vector_cable5);
    ideal_cl6 = norm(vector_cable6);
    ideal_cl7 = norm(vector_cable7);
    ideal_cl8 = norm(vector_cable8);

    % Unit cable vectors
    unit_vector_cable1 = vector_cable1 / ideal_cl1;
    unit_vector_cable2 = vector_cable2 / ideal_cl2;
    unit_vector_cable3 = vector_cable3 / ideal_cl3;
    unit_vector_cable4 = vector_cable4 / ideal_cl4;
    unit_vector_cable5 = vector_cable5 / ideal_cl5;
    unit_vector_cable6 = vector_cable6 / ideal_cl6;
    unit_vector_cable7 = vector_cable7 / ideal_cl7;
    unit_vector_cable8 = vector_cable8 / ideal_cl8;

    % Jacobian matrix
    jaco_trans = [unit_vector_cable1, unit_vector_cable2, unit_vector_cable3, ...
                  unit_vector_cable4, unit_vector_cable5, unit_vector_cable6, ...
                  unit_vector_cable7, unit_vector_cable8; ...
                  cross(vector_ee1, unit_vector_cable1), ...
                  cross(vector_ee2, unit_vector_cable2), ...
                  cross(vector_ee3, unit_vector_cable3), ...
                  cross(vector_ee4, unit_vector_cable4), ...
                  cross(vector_ee5, unit_vector_cable5), ...
                  cross(vector_ee6, unit_vector_cable6), ...
                  cross(vector_ee7, unit_vector_cable7), ...
                  cross(vector_ee8, unit_vector_cable8);];
    jaco = jaco_trans';



    ideal_cl = [ideal_cl1; ideal_cl2; ideal_cl3; ideal_cl4; ...
                ideal_cl5; ideal_cl6; ideal_cl7; ideal_cl8;];

    % Fa = -M0_ee*a_trj(:,i_step); %inertial force and moment
    force_ee = [0;0;-mass_ee*9.8] ; %external force+ Fa(1:3)

    % omega = omega_local(:, i_step);
    % alpha = alpha_local(:, i_step);
    % moment_ee = -Tr_matrix(1:3,1:3)*Iee*alpha - Tr_matrix(1:3,1:3)*Skew_F(omega)*Iee*omega; %external moment;Fa(4:6)

    moment_ee = zeros(3,1);
  

    % barycenter method
    % [ideal_cf, num_v] = bary_center(force_ee, moment_ee, jaco, force_min, force_max);
    % i_num = i_num + 1;
    % num_vertex(i_num) = num_v;



    % [ideal_cf,ideal_cf_exitflag]= pseudo_inverse_new(force_ee, moment_ee, jaco, force_min, force_max);
    % [ideal_cf, num_v] = bary_center_plot(force_ee, moment_ee, jaco, force_min, force_max);


    ij_indx = [6; 8];
    % [force_test1,~] = calculate_CFEL(force_ee, moment_ee, jaco, force_min, force_max,ij_indx);
    % [force_test2,~] = calculate_CFEL_Force(force_ee, moment_ee, jaco, force_min, force_max,ij_indx);
    [force_test3,~,CFEL,ideal_cf_exitflag] = calculate_CFEL_Force_geom(force_ee, moment_ee, jaco, force_min, force_max,ij_indx);
    

    % if ideal_cf_exitflag == -1
    %     error('绳力不满足上下限');
    % end
    % if (ideal_cf_exitflag >= 0)
    %     ideal_workspace_all(i_step) = {pose_ee};
    %     ideal_cable_length_all{i_step} = ideal_cl;
    %     ideal_cable_force_all{i_step} = ideal_cf;
    %     is_valid(i_step) = true;
    % else
    %     ideal_workspace_all{i_step} = [];
    %     is_valid(i_step) = false;
    %     continue
    % end



    % if each cable force is bigger than 0, the position is in the workspace
    if (ideal_cf_exitflag >= 0)
        i_ideal = i_ideal + 1;
        % workspace, cable length and force are put in the cells
        ideal_workspace(:,i_ideal) = [x_step; y_step; z_step;CFEL];
        % ideal_cable_length(:,i_ideal) = ideal_cl;
        % ideal_cable_force(:,i_ideal) = ideal_cf;
        % ideal_a_g(:,i_ideal) = [a1_g;a2_g;a3_g;a4_g;a5_g;a6_g;a7_g;a8_g;ao_g;ao_z_g;a_down_g;a_up_g];
       


    else
        continue
    end
end


data = ideal_workspace(:,1:i_ideal);
plot_ideal_workspace(data)