%% 课上小实验1
% clc;
% clear all;
% close all;
% omega = (0:pi/100:pi);
% H = 1/5*(1+exp(-j.*omega)+exp(-j*2.*omega)+exp(-j*3.*omega)+exp(-j*4.*omega));
% 
% figure;
% subplot(2,1,1);
% plot(omega/pi, abs(H),'LineWidth',2);
% grid on;
% set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',1);
% title('幅频响应','FontSize',14);
% subplot(2,1,2);
% plot(omega/pi, atan2 (imag (H),real (H)),'LineWidth',2);
% grid on;
% set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',1);
% title('相频响应','FontSize',14);
% xlabel('\it\fontname{Times New Roman}f \rm / Hz','FontSize',14)

%% 课上小实验2（实验报告对应的实验）
clc;
clear all;
close all;

fs = 11025;
t = 4;              % 仿真时间
T = 1/fs:1/fs:t;
N = t*fs;

F = 0:fs/N:fs/2;

wp = 1800/(fs/2);   %通带截止频率,并对其归一化
ws = 2205/(fs/2);   %阻带截止频率,并对其归一化

alpha_p = 1;        %通带最大衰减为1db
alpha_s = 60;       %阻带最小衰减为40db

%获取阶数和截止频率
[N1 wc1] = ellipord(wp , ws , alpha_p , alpha_s);

%获得转移函数系数
[b,a] = ellip(N1,alpha_p,alpha_s,wc1,'low');

%通过系数b、a获得频率响应
freqz(b,a,11025,fs);
set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',1);
title('低通滤波器的频率响应');

%% 实验2拓展

isRecor = 0;            % 1:现场录制音频用于实验（首次运行请设1）；
                        % 0:直接读取上一次的音频用于实验。

if isRecor == 1                     % 录制一段音频
    R = audiorecorder(fs, 16 ,1) ;
    
    disp('请开始讲话');
    recordblocking(R, t);
    disp('录制结束');
    
    myspeech = getaudiodata(R);

    % 保存时域波形
    audiowrite('myspeech.wav', myspeech, fs);
else
    % 读取上一次的音频文件
    [myspeech,~] = audioread("myspeech.wav");
end

% 查看音频时域波形
figure;
subplot(211);
plot(T, myspeech);
xlabel('\it\fontname{Times New Roman}t \rm/ s');
title('无噪声的人声时域波形');
% 查看音频的频率分布
myspeech_f = fftshift(abs(fft(myspeech)))/N;

subplot(212);
plot(F,myspeech_f(N/2:N));
xlabel('\it\fontname{Times New Roman}f \rm/ Hz');
title('无噪声的人声频率分布');
% 播放不含噪声的人声音频
player = audioplayer(myspeech,fs);
playblocking(player);

%% 读取鸟叫的mp3并将其频率成分限制在2210Hz以上

[bird_t1,fs1]=audioread('bird.mp3');
bird_t1 = bird_t1(:,1);
N1 = length(bird_t1);

% 降采样至11025Hz
bird_t2 = decimate(bird_t1,4);      % 4 = fs1/fs
bird_t2 = bird_t2(1:N)*0.3;         % 截取4s

% 将其频率成分限制在2210Hz以上
[N2 wc2] = ellipord(2210/(fs/2) , 2215/(fs/2) , 1 , 40);
[b2,a2] = ellip(N2, 1, 90, wc2,'high');
bird_t2 = filter(b2,a2,bird_t2)*0.5;

% 查看时域波形
figure;
subplot(211);
plot(T,bird_t2);
xlabel('\it\fontname{Times New Roman}t \rm/ s');
title('鸟鸣声时域波形');
% 鸟叫声的频率分布
bird_f2 = fftshift(abs(fft(bird_t2)))/N;
subplot(212);
plot(F,bird_f2(N/2:N));
xlabel('\it\fontname{Times New Roman}f \rm/ Hz');
title('鸟鸣声的频率分布');
% 播放鸟叫声
player = audioplayer(bird_t2,fs);
playblocking(player);

%% 将说话声和鸟叫声混合起来
mix = myspeech + bird_t2;

% 查看时域波形
figure;
subplot(211);
plot(T,mix);
xlabel('\it\fontname{Times New Roman}t \rm/ s');
title('混合声时域波形');

% 混合声音的频率分布
mix_f = fftshift(abs(fft(mix)))/N;
subplot(212);
plot(F,mix_f(N/2:N));
xlabel('\it\fontname{Times New Roman}f \rm/ Hz');
title('混合声的频率分布');

% 播放混合声音
player = audioplayer(mix,fs);
playblocking(player);

%% 经过低通滤波
mix_low = filter(b,a,mix);

% 查看时域波形
figure;
subplot(211);
plot(T,mix_low);
xlabel('\it\fontname{Times New Roman}t \rm/ s');
title('滤波器输出时域波形');

% 滤波后的频率分布
mix_low_f = fftshift(abs(fft(mix_low)))/N;
subplot(212);
plot(F,mix_low_f(N/2:N));
xlabel('\it\fontname{Times New Roman}f \rm/ Hz');
title('滤波器输出频率分布');

% 播放滤波后的声音
player = audioplayer(mix_low,fs);
playblocking(player);


