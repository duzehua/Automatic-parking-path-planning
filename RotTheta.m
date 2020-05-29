function [ bc_line ] = RotTheta( c_line, theta , x, y)
% [ bc_line ] = RotTheta( c_line, theta , x, y)
% 线条绕某点旋转一定角度，计算旋转完成后线条的位置坐标
%   input：
%      c_line： 待旋转的线条
%      theta：  绕旋转中心点转动的角度
%      x:       旋转中心的横坐标
%      y:       旋转中心的纵坐标
%   output：
%      bc_line: 旋转完成后的线条
%   author：ManQi
%   e-mail：zehuadu@126.com

bc_line = zeros(1, 4);
R = [cos(theta), -sin(theta), 0;...
     sin(theta),  cos(theta),  0;...
     0,           0,           1];

temp = R * [c_line(1) - x; c_line(3) - y; 1];
bc_line(1) = temp(1) + x;
bc_line(3) = temp(2) + y;

temp = R * [c_line(2) - x; c_line(4) - y; 1];
bc_line(2) = temp(1) + x;
bc_line(4) = temp(2) + y;
end

