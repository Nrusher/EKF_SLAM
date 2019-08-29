function [x,P]= EKF_predict (x,P,v,g,Q,WB,dt)
%function [xn,Pn]= predict (x,P,v,g,Q,WB,dt)
%
% Inputs:
%   x, P - SLAM state and covariance
%   v, g - control inputs: velocity and gamma (steer angle)
%   Q - covariance matrix for velocity and gamma
%   WB - vehicle wheelbase
%   dt - timestep
%
% Outputs: 
%   xn, Pn - predicted state and covariance
%
% Tim Bailey 2004.

% 中间变量
s= sin(g+x(3)); c= cos(g+x(3));
vts= v*dt*s; vtc= v*dt*c;

% 计算状态变量的jacobians 矩阵  
Gv= [1 0 -vts;
     0 1  vtc;
     0 0 1];

% 计算控制变量的jacobians 矩阵
Gu= [dt*c -vts;
     dt*s  vtc;
     dt*sin(g)/WB v*dt*cos(g)/WB];
  
% 更新预测的移动机器人相关部分的协方差矩阵
P(1:3,1:3)= Gv*P(1:3,1:3)*Gv' + Gu*Q*Gu';
% 更新移动机器人与landmark相关的协方差矩阵元素
if size(P,1)>3
    P(1:3,4:end)= Gv*P(1:3,4:end);
    P(4:end,1:3)= P(1:3,4:end)';
end    

% 根据模型更新预测移动机器人的位姿
x(1:3)= [x(1) + vtc; 
         x(2) + vts;
         pi_to_pi(x(3)+ v*dt*sin(g)/WB)];

