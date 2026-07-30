clear; clc; close all;

% 依次运行两个主程序。
% 每个主程序各自生成一张统一图：刚体运动状态 + 全局角速度/角加速度。
run_as_subsolver = true;
suppress_solver_plots = false;

rigid_body_newmark;
rigid_body_rk4;
