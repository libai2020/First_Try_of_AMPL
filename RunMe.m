% ==============================================================================
%  MATLAB Source Codes for the book "Cooperative Dedcision and Planning for
%  Connected and Automated Vehicles" published by Mechanical Industry Press
%  in 2020.
% 《智能网联汽车协同决策与规划技术》书籍配套代码
%  Copyright (C) 2020 Bai Li
%  2020.01.27
% ==============================================================================
%  第二章. 2.3.5小节. 调用AMPL指令求解简单的车辆轨迹规划问题，并可视化呈现优化结果
% ==============================================================================
%  补充说明：
%  1. 从Figure 3结果来看，使用10个有限元会造成直观可见的有限元非配置点误差；
%     根据经验，在K = 3前提下，一般应设置Nfe使其保证tf / Nfe <= 1.0.
% ==============================================================================
close all
% 从Matlab调用AMPL终端，并在其中执行用于轨迹规划的批处理过程 %
!ampl simple_case.run

% 导入AMPL求解得到的决策变量 %
load x.txt
load y.txt
load theta.txt
load v.txt
load a.txt
load phy.txt
load w.txt
load tf.txt

% 重采样参数设置 %
K = 3;
Nfe = length(x) / (K + 1);
precision_level = 0.01;
[~, precise_x] = GeneratePreciseProfileFromCollocationGrids(x, Nfe, tf, precision_level, 0);
[~, precise_y] = GeneratePreciseProfileFromCollocationGrids(y, Nfe, tf, precision_level, 0);
[timeline_w, precise_w] = GeneratePreciseProfileFromCollocationGrids(w, Nfe, tf, precision_level, 1);
[timeline_v, precise_v] = GeneratePreciseProfileFromCollocationGrids(v, Nfe, tf, precision_level, 0);

% 绘制重采样轨迹 %
% 其他变量的绘制方式与此相同 %
figure(1)
set(0,'DefaultLineLineWidth',3);
plot(precise_x,precise_y,'k');
axis equal; box on; grid on; axis tight;
xlabel('x axis / m','Fontsize',16,'FontWeight','bold');
ylabel('y axis / m','Fontsize',16,'FontWeight','bold');
set(gca,'FontSize',12,'FontWeight','bold');
title('车辆行驶轨迹');

figure(2)
set(0,'DefaultLineLineWidth',2);
plot(timeline_v, precise_v,'k');
box on; grid on;
axis([0 tf -3 3]);
hold on
plot(timeline_v, ones(1,length(precise_v)).*2.0, 'r');
plot(timeline_v, ones(1,length(precise_v)).*-2.0, 'r');
xlabel('Time /sec','Fontsize',16,'FontWeight','bold');
ylabel('Velocity [m/s]','Fontsize',16,'FontWeight','bold');
set(gca,'FontSize',12,'FontWeight','bold');
title('车辆运动速度v(t)');

figure(3)
set(0,'DefaultLineLineWidth',2);
plot(timeline_w, precise_w,'k');
axis([0 tf -1 1]);
box on; grid on;
hold on
plot(timeline_w, ones(1,length(precise_w)).*0.54, 'r');
plot(timeline_w, ones(1,length(precise_w)).*-0.54, 'r');
xlabel('Time /s','Fontsize',16,'FontWeight','bold');
ylabel('Derivative of Steering Angle [rad/s]','Fontsize',16,'FontWeight','bold');
set(gca,'FontSize',12,'FontWeight','bold');
title('车辆运动前轮转角角速度w(t)');