function [ c_line ] = car_p( c_l, c_w, c_o)
% [ c_line ] = car_p( c_l, c_w, c_o)
% 输入矩形长宽及对角线交点，返回矩形四顶点坐标
%   input：
%      c_l： 矩形长
%      c_w： 矩形宽
%      c_o:  对角线交点
%   output：
%      c_line: 矩形顶点信息
%   author：ManQi
%   e-mail：zehuadu@126.com

x = c_o(1);
y = c_o(2);
c_line(1,:) = [x,       x,       y - c_w/2, y + c_w/2];
c_line(2,:) = [x,       x + c_l, y + c_w/2, y + c_w/2];
c_line(3,:) = [x + c_l, x + c_l, y + c_w/2, y - c_w/2];
c_line(4,:) = [x + c_l, x,       y - c_w/2, y - c_w/2];

end

