% 仿真配置文件
% 
% 纪友州 2019

% 数据生成控制参数
N_LANDMARKS = 60; % 随机产生landmark的数目
BORDER_LENGTH = 30; % 随机产生landmark的边界大小范围
SAVE_GIF = 0; % 0不保存gif动画，1保存gif动画
NUMBER_LOOPS= 1; % 跑的圈数
AT_WAYPOINT= 0.5; % 判定切换下一点的距离
SWITCH_CONTROL_NOISE = 1; % 是否添加控制噪声
SWITCH_SENSOR_NOISE = 1; % 是否添加观测噪声
ADD_COLOR_NOISE = 0;
% c = [1 0.8 -0.6 0.5 -0.4 0.2];
c = [1 0.8 -0.6];

% 仿真文件控制参数
SLAM_SAVE_GIF = 1; % 0不保存gif动画，1保存gif动画
MAP_DATA = 'map3.mat';
REDUCE_OB_FEATURES = 0;
ASYNCHRONOUS = 0; % 是否同步
SWITCH_ASSOCIATION_KNOWN= 0; % id是否已知
GATE_REJECT= 4.0; % 判断为已知特征的最大距离
GATE_AUGMENT= 25.0; % 判断为新特征的最小距离
SWITCH_USE_IEKF = 1;

% 控制参数
V= 8; % m/s，速度
MAXG= 30*pi/180; % rad，最大的方向角
RATEG= 20*pi/180; % rad/s, 最大转向速率
WHEELBASE= 4; % metres, 轮距
DT_CONTROLS= 0.025; % 控制频率

% 控制噪声参数
sigmaV= 2; % m/s
sigmaG= (10.0*pi/180); % radians
Q= [sigmaV^2 0; 0 sigmaG^2];

% 观测参数
MAX_RANGE= 30.0; % metres 最大观测距离
DT_OBSERVE= 8*DT_CONTROLS; % seconds, 观测频率

% 观测噪声
sigmaR= 0.1; % metres
sigmaB= (1.0*pi/180); % radians
R= [sigmaR^2 0; 0 sigmaB^2];

