function wrench_E = transform_sensor_wrench_to_ee(wrench_sensor_raw_S, R_ES, r_ES_E, sensor_sign)
%TRANSFORM_SENSOR_WRENCH_TO_EE 将传感器原始六维力转换到动平台质心 local frame
%   R_ES: sensor frame 到 EE local frame 的旋转矩阵
%   r_ES_E: sensor 原点相对 EE 质心的位置，在 EE local frame 下表达

if nargin < 4 || isempty(sensor_sign)
    sensor_sign = 1;
end

wrench_sensor_raw_S = wrench_sensor_raw_S(:);
r_ES_E = r_ES_E(:);

F_S = sensor_sign * wrench_sensor_raw_S(1:3);
M_S = sensor_sign * wrench_sensor_raw_S(4:6);

F_E = R_ES * F_S;
M_E_at_sensor = R_ES * M_S;
M_E_at_com = M_E_at_sensor + Skew_F(r_ES_E) * F_E;

wrench_E = [F_E; M_E_at_com];
end
