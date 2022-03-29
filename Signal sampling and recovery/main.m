% 下面的代码分为1、2.1、2.2、2.3、3五个节，其中1节代码可修改Ts为pi、1.3*pi、0.5*pi后分别运行
% 对应运行2.1、2.2或2.3（2.X只能运行其中某一个），分别对应临界采样、欠采样和过采样的分析
% 3节代码是通用的，用于频谱分析
%% 1 参数初始化
clc;
clear all;
close all;

omega_m = 1;    % 信号带宽
omega_c = 1;    % 滤波器截止频率

Ts = 0.05*pi;        % 临界采样pi、欠采样1.3*pi、过采样0.5*pi
Tp = 40;        % 仿真时长

N = ceil(Tp/Ts) % 采样点数
nTs = -20:Tp/(N-1):20;  % 时域采样时刻

t = -20:0.005:20;   % 恢复间隔为0.005s

f_nTs = sinc(nTs/pi);   % 采样信号

ins = Ts*omega_c/pi*sinc((omega_c/pi)* ...
    (ones(length(nTs),1)*t-nTs'*ones(1,length(t))));    % 内插信号

f_a = f_nTs*ins;         % 恢复信号

error = abs(f_a-sinc(t/pi));

%% 2.1下面是临界采样画图的操作
figure;
subplot(211);
plot(t,sinc(t/pi),'b-','LineWidth',2);   % 原始信号
hold on;
stem(nTs,f_nTs,'g--','LineWidth',2);     % 采样信号
grid on;
legend('FontSize',14);
legend('原始信号','采样信号');
set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',1);
title('信号采样','FontSize',14);
xlabel('\it\fontname{Times New Roman}t \rm/ s','FontSize',14);


subplot(212);
stem(nTs,f_nTs,'g--','LineWidth',1.5);      % 采样信号
hold on;

plot(t,f_nTs(6)*ins(6,:),'b--','LineWidth',1.5);   % 原始信号
hold on;
plot(t,f_nTs(7)*ins(7,:),'k--','LineWidth',1.5);   % 原始信号
hold on;
plot(t,f_nTs(8)*ins(8,:),'--','LineWidth',1.5,'Color',[0.3 0.8 0.9]);   % 原始信号
hold on;
plot(t,f_a,'r','LineWidth',2);          % 恢复信号
hold on;

plot(nTs(6),f_nTs(6),'bx');      % 采样信号
hold on;
plot(nTs(7),f_nTs(7),'kx');      % 采样信号
hold on;
plot(nTs(8),f_nTs(8),'x','Color',[0.3 0.8 0.9]);      % 采样信号


legend('FontSize',14);
legend('采样信号','\it\fontname{Times New Roman}f_{a}^{\rm (6)}','\it\fontname{Times New Roman}f_{a}^{\rm (7)}','\it\fontname{Times New Roman}f_{a}^{\rm (8)}','恢复信号');
grid on;
set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',1);
title('信号重建','FontSize',14);
xlabel('\it\fontname{Times New Roman}t \rm/ s','FontSize',14);

figure;
plot(t,error,'b-','LineWidth',2);
grid on;
set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',1);
title('重建误差','FontSize',14);
% axis([-20 20 0 0.5])
xlabel('\it\fontname{Times New Roman}t \rm/ s','FontSize',14);

%% 2.2 下面是欠采样画图的操作
figure;
subplot(211);
plot(t,sinc(t/pi),'b-','LineWidth',2);   % 原始信号
hold on;
stem(nTs,f_nTs,'g--','LineWidth',2);     % 采样信号
grid on;
legend('FontSize',14);
legend('原始信号','采样信号');
set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',1);
title('信号采样','FontSize',14);
xlabel('\it\fontname{Times New Roman}t \rm/ s','FontSize',14);


subplot(212);
stem(nTs,f_nTs,'g--','LineWidth',1.5);      % 采样信号
hold on;

plot(t,f_nTs(5)*ins(5,:),'b--','LineWidth',1.5);   % 原始信号
hold on;
plot(t,f_nTs(6)*ins(6,:),'--','LineWidth',1.5,'Color',[0.3 0.8 0.9]);   % 原始信号
hold on;
plot(t,f_a,'r','LineWidth',2);          % 恢复信号
hold on;

plot(nTs(5),f_nTs(5),'bx');      % 采样信号
hold on;
plot(nTs(6),f_nTs(6),'x','Color',[0.3 0.8 0.9]);      % 采样信号



legend('FontSize',14);
legend('采样信号','\it\fontname{Times New Roman}f_{a}^{\rm (5)}','\it\fontname{Times New Roman}f_{a}^{\rm (6)}','恢复信号');
grid on;
set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',1);
title('信号重建','FontSize',14);
xlabel('\it\fontname{Times New Roman}t \rm/ s','FontSize',14);

figure;
plot(t,error,'b-','LineWidth',2);
grid on;
set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',1);
title('重建误差','FontSize',14);
% axis([-20 20 0 0.5])
xlabel('\it\fontname{Times New Roman}t \rm/ s','FontSize',14);
%% 2.3 下面是过采样画图的操作
figure;
subplot(211);
plot(t,sinc(t/pi),'b-','LineWidth',2);   % 原始信号
hold on;
stem(nTs,f_nTs,'g--','LineWidth',2);     % 采样信号
grid on;
legend('FontSize',14);
legend('原始信号','采样信号');
set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',1);
title('信号采样','FontSize',14);
xlabel('\it\fontname{Times New Roman}t \rm/ s','FontSize',14);


subplot(212);
stem(nTs,f_nTs,'g--','LineWidth',1.5);      % 采样信号
hold on;

plot(t,f_nTs(12)*ins(12,:),'b--','LineWidth',1.5);   % 原始信号
hold on;
plot(t,f_nTs(13)*ins(13,:),'k--','LineWidth',1.5);   % 原始信号
hold on;
plot(t,f_nTs(14)*ins(14,:),'--','LineWidth',1.5,'Color',[0.3 0.8 0.9]);   % 原始信号
hold on;
plot(t,f_a,'r','LineWidth',2);          % 恢复信号
hold on;

plot(nTs(12),f_nTs(12),'bx');      % 采样信号
hold on;
plot(nTs(13),f_nTs(13),'kx');      % 采样信号
hold on;
plot(nTs(14),f_nTs(14),'x','Color',[0.3 0.8 0.9]);      % 采样信号


legend('FontSize',14);
legend('采样信号','\it\fontname{Times New Roman}f_{a}^{\rm (12)}','\it\fontname{Times New Roman}f_{a}^{\rm (13)}','\it\fontname{Times New Roman}f_{a}^{\rm (14)}','恢复信号');
grid on;
set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',1);
title('信号重建','FontSize',14);
xlabel('\it\fontname{Times New Roman}t \rm/ s','FontSize',14);

figure;
plot(t,error,'b-','LineWidth',2);
grid on;
set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',1);
title('重建误差','FontSize',14);
% axis([-20 20 0 0.5])
xlabel('\it\fontname{Times New Roman}t \rm/ s','FontSize',14);
%% 3 频域分析
figure;
xa = sinc(t/pi);
fs1 = 200;
N1 = length(t);
F1 = 0:fs1/N1:fs1/2;
xa_f = fftshift(abs(fft(xa)))/N1;
plot(F1,xa_f(N1/2:N1),'LineWidth',1.5);
hold on;
f_a_f = fftshift(abs(fft(f_a)))/N1;
plot(F1,f_a_f(N1/2:N1),'LineWidth',1.5);
legend('原始信号频谱','重建信号频谱','FontSize',14);
xlabel('\it\fontname{Times New Roman}f \rm / Hz','FontSize',14)
axis([0 0.5 0 0.13]);
grid on;
set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',1);
