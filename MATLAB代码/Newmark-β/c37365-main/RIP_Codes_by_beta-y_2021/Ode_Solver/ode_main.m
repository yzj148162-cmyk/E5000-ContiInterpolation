%% Copyright © 2021 <beta-y> %%

%% ode 初始化
T_stop=10;% 仿真时长 s
T_sample = 0.002;% 采样周期 s
x0 = [0;0;1/2*pi;0];% 初始状态 [q0;dq0;q1;dq1]; 期望状态 x_f = [0;0;0;0];
options=odeset('RelTol',1.0e-6,'AbsTol',1.0e-6,'BDF','on');%设置仿真精度,Backward Differentiation Formulas (BDFs) instead of the default Numerical Differentiation Formulas (NDFs).

%% ode 求解
[t,x]=ode45(@(t,x) RIP_Sysm(t,x),[0:T_sample:T_stop],x0,options); % ode45解常微分方程

%% 绘图
figure(1)
plot(t,x(:,1));title('时间-旋臂角度');
xlabel('t[s]');ylabel('\theta_0[rad]')
figure(2)
plot(t,x(:,3));title('时间-摆杆角度');
xlabel('t[s]');ylabel('\theta_1[rad]')

%% 待积分函数
function dx = RIP_Sysm(t,x) 
	%% Constants 
	L_0 = 0.2159; %Total lenth of the arm
	l_1 = 0.1556; % Distance from pivot to center of pendulum's mass
	m_1 = 0.1270; % Mass of pendulum
	J_0 = 0.0020; % Inertia of the arm about the rotation axis
	J_1 = 0.0012; % Pendulum moment of intertia about center of mass
	g = 9.80;     % Acc of the gravity
	
	%% D(q) 正定惯性矩阵
	D_11 = J_0 + m_1*(L_0^2+l_1^2*sin(x(3))^2);  
	D_12 = m_1*l_1*L_0*cos(x(3));
	D_21 = D_12;
	D_22 = J_1 + m_1*l_1^2;
	D=[D_11,D_12;D_21,D_22];
	D_inv =inv(D);
	
	% C(q,dq) 离心力和哥氏力
	C_11 = 1/2*m_1*l_1^2*sin(2*x(3))*x(4);
	C_12 = -m_1*l_1*L_0*sin(x(3))*x(4)+1/2*m_1*l_1^2*sin(2*x(3))*x(2);
	C_21 = -1/2*m_1*l_1^2*sin(2*x(3))*x(2);
	C_22 = 0;
	C = [C_11,C_12;C_21,C_22];
	
	% G(q) 重力
	G = [0; -m_1*g*l_1*sin(x(3))];
	
    U = [0;0];%零输入响应
    
	dq=[x(2);x(4)];    % 角度 x1的导数，x2的导数
	ddq= D_inv*(U - C*dq - G );  % 动力学方程反解角加速度：D(q)*ddq + C*dq + G = U
	dx = [dq(1);ddq(1);dq(2);ddq(2)];

end