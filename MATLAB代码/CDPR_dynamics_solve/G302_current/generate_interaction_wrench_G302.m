function sensor_data = generate_interaction_wrench_G302(t_step, N_total, scenario)
%GENERATE_INTERACTION_WRENCH_G302 生成假定六维力传感器原始数据
%   wrench_raw_S = [Fx; Fy; Fz; Mx; My; Mz]，均在 sensor frame 下表达。

if nargin < 3 || isempty(scenario)
    scenario = 'default';
end

N_total = round(N_total);          % 积分区间数
t_sample = (0:N_total) * t_step;   % 传感器采样时间点，共 N_total+1 个
wrench_raw_S = zeros(6, N_total+1);

switch lower(scenario)
    case 'default'
        % 小幅有界振荡交互力，用于第一版验证动力学递推链路。
        wrench_raw_S(1,:) = 8 * cos(2*pi*0.2*t_sample);
        % wrench_raw_S(2,:) = 0.4 * cos(2*pi*0.2*t);
        % wrench_raw_S(6,:) = 0.02 * cos(2*pi*0.2*t);
        

    case 'zero'
        wrench_raw_S(:,:) = 0;

    otherwise
        error('未知的交互力场景: %s', scenario);
end

sensor_data.N_total = N_total;
sensor_data.t_sample = t_sample;
sensor_data.wrench_raw_S = wrench_raw_S;
sensor_data.scenario = scenario;
end
