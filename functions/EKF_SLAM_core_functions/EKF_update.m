function [x,P]= EKF_update(x,P,z,R,idf, batch)
% function [x,P]= update(x,P,z,R,idf, batch)
%
% Inputs:
%   x, P - SLAM state and covariance
%   z, R - range-bearing measurements and covariances
%   idf - feature index for each z
%   batch - switch to specify whether to process measurements together or sequentially
%
% Outputs:
%   x, P - updated state and covariance

if size(z,2) == 0
    x = x;
    P = P;
    return
end

if batch == 1
    [x,P]= batch_update(x,P,z,R,idf);
else
    [x,P]= single_update(x,P,z,R,idf);
end

%
%
% 批次更新
function [x,P]= batch_update(x,P,z,R,idf)

lenz= size(z,2);
lenx= length(x);
H= zeros(2*lenz, lenx);
v= zeros(2*lenz, 1);
RR= zeros(2*lenz);

% 将所有的观测值放到同一矩阵中
for i=1:lenz
    ii= 2*i + (-1:0);
    [zp,H(ii,:)]= observe_model(x, idf(i));
    
    v(ii)=      [z(1,i)-zp(1);
        pi_to_pi(z(2,i)-zp(2))];
    RR(ii,ii)= R;
end

% 利用所有数据一次性更新
[x,P]= KF_cholesky_update(x,P,v,RR,H);

%  单个更新
function [x,P]= single_update(x,P,z,R,idf)

% 获得多少个路标
lenz= size(z,2);

% 依次更新路标进行更新
for i=1:lenz
    % 根据观测模型获得观测模型预测观测值
    [zp,H]= observe_model(x, idf(i));
    
    % 获取预测观测值与实际观测值之间的差值
    v= [z(1,i)-zp(1);
        pi_to_pi(z(2,i)-zp(2))];
    
    % 获得更新值
    [x,P]= KF_cholesky_update(x,P,v,RR,H);
end        
