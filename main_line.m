clc;clear;close all
%   main_line.m
%   һ���򵥵Ľű�ʵ���Զ�������·���滮��̬��ʾ�����Բ���·�߽���������ϡ�
%   author��ManQi
%   e-mail��zehuadu@126.com
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% �����趨
up_limit = 6;%�ϱ߽�������
front_p = [5, 2]; % ��λ��ϢC��, �μ�˵���ĵ������ײ���ڶ�����
right_limit = 11; % �������ұ߽�

o_3 = [0.5, 1]; % ������
ini = [6, 4];   % ��ʼ��

c_l = 3; % ����
c_w = 1.4; % ����
r_min = 2;    % ��Сת��뾶
plot_vec = 0.01; % �����ٶ�, ��ԽС, Խ��, Ϊ0ֱ�ӳ�ͼ
fit_model = 0; % �����ʽ, 1λ�ߴζ���ʽ, ������Ϊ����Ҷ�������, ͼ�������ֵĺ�ɫ����Ϊ�������
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(1)
hold on
grid on
%% ���������޶�
% ���½�����, ��x����, ��y����
line([0, right_limit],          [up_limit, up_limit]    ,'LineWidth',2, 'Color',[0 0 0]);
line([0, front_p(1)],           [0, 0]                  ,'LineWidth',2, 'Color',[0 0 0]);
line([front_p(1), front_p(1)],  [0, front_p(2)]         ,'LineWidth',2, 'Color',[0 0 0]);
line([front_p(1), right_limit], [front_p(2), front_p(2)],'LineWidth',2, 'Color',[0 0 0]);
axis([0 right_limit 0 up_limit], 'equal');
%% ����
x = ini(1); y = ini(2); % �����������ĵ�����
c_o = [x, y];

c_line(1,:) = [x,       x,       y - c_w/2, y + c_w/2];
c_line(2,:) = [x,       x + c_l, y + c_w/2, y + c_w/2];
c_line(3,:) = [x + c_l, x + c_l, y + c_w/2, y - c_w/2];
c_line(4,:) = [x + c_l, x,       y - c_w/2, y - c_w/2];

for i = 1:4
    car(i) = line([c_line(i,1),c_line(i,2)], [c_line(i,3), c_line(i,4)],'LineWidth',1.5, 'Color',[0 0 0]);
end
%% ����ȷ��
% ���Ʒ�, ������Сת��뾶ȷ����2��Բ��
cir2 = [o_3(1), o_3(2) + r_min];
% plot(cir2(1), cir2(2), 'ro');

% ���ݹ���ʼ��Ͳ������Բ��Բ��ȷ�����뾶
k_1 = (c_o(2) - o_3(2)) / (c_o(1) - o_3(1));
k_2 = - 1/k_1;
r_max = y - (k_2 * x + ((c_o(2) + o_3(2))/2 - k_2*(c_o(1) + o_3(1))/2)); % ��һ��ת�����뾶

% ��1, ���������Сת��뾶ȷ����Բ��
p_rmin = [x, y - r_min];
p_rmax = [x, y - r_max];

% ��Сת��뾶��Ӧ�ĵ�Բ��2-r_min�ľ���͵���ʼ��ľ���, �Լ�����Ӧ��ת��
v_min = p_rmin - cir2;
l_min = norm(v_min) - r_min; % ��Сת��뾶ʱ, ��o_3���ߵõ��İ뾶�Ա�
d_min = y - p_rmin(2);
alpha_min = pi/2 - atan2(abs(v_min(2)), abs(v_min(1)));
% alpha_min*180/pi

% ���ת��뾶��Ӧ�ĵ�Բ��2-r_min�ľ���͵���ʼ��ľ���, �Լ�����Ӧ��ת��
v_max = p_rmax - cir2;
l_max = norm(v_max) - r_min;
d_max = y - p_rmax(2);
alpha_max = pi/2 - atan2(abs(v_max(2)), abs(v_max(1)));
% alpha_max*180/pi

% 2�ַ����������е���������, ��1�İ뾶��Բ��
l_middle = inf;
d_middle = 0;
% ���ڻ�1����Сת��뾶��ӦԲ�Ĺ��ɵĵ���ʼ��ĳ�һ��С�ڵ�Բ��2�ĵ�, ͬ��, ��һ���෴. �Դ��ж�����, �Ƿ����㲴�����Ҫ��.
if (l_min > d_min) && (l_max < d_max)
    while abs(l_middle - d_middle) > 0.000001
        % �����м�Ƕȶ�Ӧ��������x = x������ΪԲ��, ����l��d
        alpha_middle = (alpha_max + alpha_min)/2;
        k_middle = -tan(pi/2 - alpha_middle);
        y_middle = k_middle*c_o(1) + (cir2(2) - k_middle*cir2(1));
        d_middle = y - y_middle;
        v_middle = [x, y_middle] - cir2;
        l_middle = norm(v_middle) - r_min;
        
        if l_middle > d_middle
            v_min = [x, y_middle] - cir2;
            l_min = norm(v_min) - r_min; % ��Сת��뾶ʱ, ��o_3���ߵõ��İ뾶�Ա�
            d_min = y - y_middle;
            alpha_min = pi/2 - atan2(abs(v_min(2)), abs(v_min(1)));
        else
            v_max = [x, y_middle] - cir2;
            l_max = norm(v_max) - r_min;
            d_max = y - y_middle;
            alpha_max = pi/2 - atan2(abs(v_max(2)), abs(v_max(1)));
        end
    end
    %     plot(x, y_middle, 'ro')
    d_middle = d_max;
    alpha = alpha_max;
else
    disp('·���滮ʧ��, ��1�뾶�������ת��뾶(��С����Сת��뾶)!');
    return
end
%% ��ɢ���ݹ���
% ����Ϣ
cir1_r = d_middle;
cir1_o = [x, y_middle];

cir2_r = r_min;
cir2_o = [o_3(1), o_3(2) + r_min];

% ��������
alpha_arr = linspace(0, alpha, 50);
alpha_arr = alpha_arr';
cir1_data = [x - d_middle.*sin(alpha_arr), y_middle + d_middle.*cos(alpha_arr), alpha_arr];
alpha_arr = linspace(alpha, 0, 50);
alpha_arr = alpha_arr';
cir2_data = [cir2_o(1) + r_min.*sin(alpha_arr), cir2_o(2) - r_min.*cos(alpha_arr), alpha_arr];

cir_data = [cir1_data; cir2_data];

% for i = 1:length(cir_data)
%     plot(cir_data(i,1), cir_data(i,2),'r.');
% end

% return
%% ��ײ�ж�
% ��1Բ��cir1_o, ��·�ϱ߽�up_limit
% ����ǰ����1Բ�ĵľ���
d_co1 = sqrt(c_l^2 + (c_w/2 + d_middle)^2);
theta_co1 = atan2((c_w/2 + d_middle), c_l);
if theta_co1 < alpha
    if up_limit - cir1_o(2) < d_co1
        disp('����ǰͷ���ϱ߽�, ·���滮ʧ��, �볢�Խ���ʼ�������ƶ�, ���С��Сת��뾶!');
        return;
    end
else
    if up_limit - cir1_o(2) < d_co1*cos(theta_co1 - alpha)
        disp('����ǰͷ���ϱ߽�, ·���滮ʧ��, �볢�Խ���ʼ�������ƶ�, ���С��Сת��뾶!');
        return;
    end
end

% ��2Բ��cir2_o, ��λǰ�߽�front_limit
% ����ǰ����2Բ�ĵľ���
d_co2 = sqrt(c_l^2 + (c_w/2 + r_min)^2);
theta_co2 = atan2((c_w/2 + r_min), c_l);
if theta_co2 + alpha >= pi/2
    if front_p(1) - cir2_o(1) < d_co2*sin(pi/2 - atan2(abs(front_p(2) - cir2(2)), abs(front_p(1) - cir2(1))))
        disp('����ǰͷ��ǰ�߽�, ·���滮ʧ��, �볢�Լ�С��Сת��뾶, �򽫲����������ƶ�!');
        return;
    end
else
    if front_p(1) - cir2_o(1) < d_co2*sin(theta_co2 + alpha)
        disp('����ǰͷ��ǰ�߽�, ·���滮ʧ��, �볢�Լ�С��Сת��뾶, �򽫲����������ƶ�!');
        return;
    end
end
%% ����
for item = 1:length(cir_data)
    
    c_o = [cir_data(item, 1), cir_data(item, 2)];
    plot(cir_data(item, 1), cir_data(item, 2), 'g.');
    
    c_line = car_p(c_l, c_w, c_o);
    for i = 1:4
        bc_line = RotTheta( c_line(i,:), cir_data(item, 3) , c_o(1), c_o(2));
        set(car(i), 'XData', [bc_line(1) bc_line(2)],'YData', [bc_line(3) bc_line(4)]);
    end
    pause(plot_vec)
end

o_3 = [0.5, 1]; % ������
ini = [6, 4];   % ��ʼ��
%% ���
if fit_model == 1
    xdata = [linspace(ini(1)+1,ini(1),100)'; cir_data(:,1); linspace(o_3(1),0,100)'];
    ydata = [linspace(ini(2),ini(2),100)'; cir_data(:,2); linspace(o_3(2),o_3(2),100)'];
    
    fit_k = polyfit(xdata, ydata, 9);
    x_data = linspace(o_3(1),ini(1),100);
    for i = 1:100
        x = x_data(i);
        y(i) = fit_k(1)*x^9 + fit_k(2)*x^8 + fit_k(3)*x^7 + fit_k(4)*x^6 + fit_k(5)*x^5 + fit_k(6)*x^4 + fit_k(7)*x^3 + fit_k(8)*x^2 + fit_k(9)*x + fit_k(10);
    end
    plot(x_data, y, 'r');
else
    %% fourier
    xdata = [linspace(ini(1)+1,ini(1),100)'; cir_data(:,1); linspace(o_3(1),0,100)']';
    ydata = [linspace(ini(2),ini(2),100)'; cir_data(:,2); linspace(o_3(2),o_3(2),100)']';
    
    L = - ini(1) - 1;
    m = 20;
    for n=1:m
        bn(n)=(2/L)*trapz(xdata,ydata.*sin(2*n*pi*xdata/L));
        an(n)=(2/L)*trapz(xdata,ydata.*cos(2*n*pi*xdata/L));
    end
    
    a0=1/L*trapz(xdata,ydata);
    x_fit=linspace(ini(1)+1,0,100);
    y_fit = zeros(1,100);
    for n=1:m
        y_fit=y_fit+bn(n)*sin(2*n*pi/L*x_fit)+an(n)*cos(2*n*pi/L*x_fit);
    end
    y_fit=y_fit+a0;
    plot(x_fit, y_fit, 'r');
end

