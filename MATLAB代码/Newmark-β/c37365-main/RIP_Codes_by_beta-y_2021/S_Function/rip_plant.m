%% Copyright © 2021 <beta-y> %%

%-------S-Function主入口函数,实现状态迁移--------%                                                                                                                                                                                      -----%
function [sys,x0,str,ts] = rip_plant(t,x,u,flag)
    % 判断 flag
    switch flag  
        % Initialization
          case 0     % flag == 0，执行 mdlInitializeSizes 子函数
            [sys,x0,str,ts]=mdlInitializeSizes();
          case 1     % flag == 1，执行 mdlDerivatives 子函数
            sys= mdlDerivatives(t,x,u);
        % Outputs
          case 3     % flag == 3，执行 mdlOutputs 子函数
            sys=mdlOutputs(t,x,u);
        % Unhandled flags
          case {2, 4, 9 }
            sys = [];
        % Unexpected flags
          otherwise
            error(['Unhandled flag = ',num2str(flag)]);
    end
end

%-------mdlInitializeSizes 函数：初始化函数--------%
function [sys,x0,str,ts] = mdlInitializeSizes()
    sizes = simsizes;
    sizes.NumContStates  = 4;   % 连续状态变量 4 个  (θ0,dθ0,θ1,dθ1)
    sizes.NumDiscStates  = 0;   % 离散状态变量 0 个
    sizes.NumOutputs     = 4;   % 输出量 4 个     (θ0,dθ0,θ1,dθ1)
    sizes.NumInputs      = 1;   % 输入量 1 个     旋臂电机驱动力矩τ
    sizes.DirFeedthrough = 0;   % 表征：输入不控制输出
    sizes.NumSampleTimes = 1;   % 表征：被控对象为连续采样
    sys=simsizes(sizes);

    x0=[0;0;1/2*pi;0];% 初始状态
    str=[];
    ts=[0 0]; % 表征：被控对象为连续采样
end

%-------mdlDerivatives 函数:计算连续状态导数-------%
function sys=mdlDerivatives(t,x,u)
    %% Constants
    L_0 = 0.2159; %0.126;%Total lenth of the arm
    l_1 = 0.1556; %0.156;% Distance from pivot to center of pendulum's mass
    m_1 = 0.1270; %0.127;% Mass of pendulum
    J_0 = 0.0020; %0.0020;% Inertia of the arm about the rotation axis
    J_1 = 0.0012; %0.0012;% Pendulum moment of intertia about center of mass
    g = 9.80;     % Acc of the gravity

    %% M(q) 正定惯性矩阵
    D_11 = J_0 + m_1*(L_0^2+l_1^2*sin(x(3))^2);  
    D_12 = m_1*l_1*L_0*cos(x(3));
    D_21 = D_12;
    D_22 = J_1 + m_1*l_1^2;
    D=[D_11,D_12;D_21,D_22];

    %% C(q,dq) 离心力和哥氏力
    C_11 = 1/2*m_1*l_1^2*sin(2*x(3))*x(4);
    C_12 = -m_1*l_1*L_0*sin(x(3))*x(4)+1/2*m_1*l_1^2*sin(2*x(3))*x(2);
    C_21 = -1/2*m_1*l_1^2*sin(2*x(3))*x(2);
    C_22 = 0;
    C = [C_11,C_12;C_21,C_22];

    %% G(q) 重力
    G = [0; -m_1*g*l_1*sin(x(3))];

    %% U 广义力
    U=[u(1);0];      

    dq=[x(2);x(4)];                 % 角度 x1 的导数，x2的导数
    ddq=inv(D)*(U - C*dq - G);  % 动力学方程反解角加速度：D(q)*qdd + C*q + G= U

    % 这里的sys 为中间变量
    % mdlDerivatives函数里给sys赋值,意味着给状态变量x 的导数赋值,结果进行积分得到状态变量（https://www.ilovematlab.cn/thread-527933-1-1.html）
    sys(1)=dq(1);
    sys(2)=ddq(1);
    sys(3)=dq(2);
    sys(4)=ddq(2);
end

%-------mdlOutputs 函数：输出状态变量-------%
function sys=mdlOutputs(t,x,u)
    % mdlOutput函数里给sys赋值,意味着给输出量y赋值（https://www.ilovematlab.cn/thread-527933-1-1.html）
    sys(1)=x(1);  % 状态变量 x1,作为被控对象的输出，同时反馈到控制器
    sys(2)=x(2);  % 状态变量 x1的导数,作为被控对象的输出，同时反馈到控制器
    sys(3)=x(3);  % 状态变量 x2,作为被控对象的输出，同时反馈到控制器
    sys(4)=x(4);  % 状态变量 x2的导数,作为被控对象的输出，同时反馈到控制器
end