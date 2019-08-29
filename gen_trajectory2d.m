function [ data ] = gen_trajectory2d( t )
% 根据时间产生运动轨迹
% t - step
% data - 产生的 x y
% radius = 1;
% data = [cos(radius*t)+cos(2*t*radius);4*sin(radius*t)+4*sin(t*radius)];
% data = data + 16;

radius = 40;
data = [radius*cos(t);radius*sin(t)];
data = data + radius+6;
end

