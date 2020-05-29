function [ bc_line ] = RotTheta( c_line, theta , x, y)
% [ bc_line ] = RotTheta( c_line, theta , x, y)
% ������ĳ����תһ���Ƕȣ�������ת��ɺ�������λ������
%   input��
%      c_line�� ����ת������
%      theta��  ����ת���ĵ�ת���ĽǶ�
%      x:       ��ת���ĵĺ�����
%      y:       ��ת���ĵ�������
%   output��
%      bc_line: ��ת��ɺ������
%   author��ManQi
%   e-mail��zehuadu@126.com

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

