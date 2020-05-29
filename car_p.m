function [ c_line ] = car_p( c_l, c_w, c_o)
% [ c_line ] = car_p( c_l, c_w, c_o)
% ������γ����Խ��߽��㣬���ؾ����Ķ�������
%   input��
%      c_l�� ���γ�
%      c_w�� ���ο�
%      c_o:  �Խ��߽���
%   output��
%      c_line: ���ζ�����Ϣ
%   author��ManQi
%   e-mail��zehuadu@126.com

x = c_o(1);
y = c_o(2);
c_line(1,:) = [x,       x,       y - c_w/2, y + c_w/2];
c_line(2,:) = [x,       x + c_l, y + c_w/2, y + c_w/2];
c_line(3,:) = [x + c_l, x + c_l, y + c_w/2, y - c_w/2];
c_line(4,:) = [x + c_l, x,       y - c_w/2, y - c_w/2];

end

