function [G,iwp]= compute_steering(x, wp, iwp, minD, G, rateG, maxG, dt)
%function [G,iwp]= compute_steering(x, wp, iwp, minD, G, rateG, maxG, dt)
%
% INPUTS:
%   x - true position
%   wp - waypoints
%   iwp - index to current waypoint
%   minD - minimum distance to current waypoint before switching to next
%   G - current steering angle
%   rateG - max steering rate (rad/s)
%   maxG - max steering angle (rad)
%   dt - timestep
%
% OUTPUTS:
%   G - new current steering angle
%   iwp - new current waypoint
%

% 判断目前是否到达设置点
cwp= wp(:,iwp);
% 计算距离
d2= (cwp(1)-x(1))^2 + (cwp(2)-x(2))^2;

if d2 < minD^2
    % 距离达到条件，前往下一个目的点
    iwp= iwp+1; 
    if iwp > size(wp,2)
        % 如果跑完一圈归零
        iwp=0;
        return;
    end
    % 更新下一个路标
    cwp= wp(:,iwp); 
end

% 计算需要调整的航角
deltaG= pi_to_pi(atan2(cwp(2)-x(2), cwp(1)-x(1)) - x(3) - G);

% 限制转向速度
maxDelta= rateG*dt;
if abs(deltaG) > maxDelta
    deltaG= sign(deltaG)*maxDelta;
end

% 限制转向角度
G= G+deltaG;
if abs(G) > maxG
    G= sign(G)*maxG;
end
