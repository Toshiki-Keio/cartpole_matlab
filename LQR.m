clear; clc; 
close all;
%% Simulation parameters of Cartpole
M=0.1;         % Mass of cart 
m=0.08;        % Mass of pendulum
J=3.20e-4;      % Inertia moment
L=0.2;          % Length
mu=2.74e-5;     % Damping coefficient
g=9.81;         % Gravity
%% State Space
M_mat = [1 0 0 0;0 1 0 0; 0 0 M+m m*L;0 0 m*L J+m*L^2];
A = inv(M_mat)*[0 0 1 0; 0 0 0 1;0 0 -mu 0;0 m*g*L 0 -mu];
B = inv(M_mat)*[0;0;1;0];
C = [1 0 0 0;0 1 0 0];
D = [0;0];
sys = ss(A,B,C,D);
%% check if system is controllable / observable
%対角正準形
[csys,T] = canon(sys,'modal');
csys

%可制御性行列
Co = ctrb(A,B);
Unco = length(A)-rank(Co)%ここが0になったら可制御

%可観測性行列
Ob = obsv(A,C);
Unob = length(A)-rank(Ob)%ここが0になったら可観測

%% Get LQR Gain
Q = diag([1e-20 1 1 1]);
R = 1;
% K = lqr(A,B,Q,R);%状態フィードバックゲイン取得
P = care(A,B,Q,R);%リカッチ方程式を解くことで行列Ｐを導出
K = inv(R)*B'*P;%状態FBゲイン導出

%% Simulation Conditions
savefig = true;

Init_angle = 0; %Poleの初期角度
Init_angvel = -40;
init_vel = 2.0;
x0 = [0 Init_angle*pi/180 0 0];


controllable = true; %制御かけるかどうか
disturbance_on = true; %外乱かけるかどうか
Tf = 10; %シミュレーション時間
control_period = 1e-3; %制御周期
Umax = 100; %アクチュエータの最大出力
%% Run simulation
sim("cartpole.slx");
%% Draw states
% figure(1)
% hold on; grid on;
% plot(output.Time,output.Data(:,1),LineWidth=2);
% plot(output.Time,output.Data(:,3),LineWidth=2);
% legend(["z","dz"]);
% title("Cart")
% xlabel("Time [s]")
% ylabel("Displacement [m], Velocity [m/s]")
% 
% figure(2)
% hold on; grid on;
% plot(output.Time,output.Data(:,2),LineWidth=2);
% plot(output.Time,output.Data(:,4),LineWidth=2);
% legend("theta","dtheta")
% title("Pole")
% xlabel("Time [s]")
% ylabel("Angle [rad], Angular Velocity [rad/s]")
% 
% figure(3)
% hold on; grid on;
% plot(control_input,LineWidth = 2);
% title("Control Input")
% xlabel("Time [s]")
% ylabel("Control Input [N]")

%% Draw estimation accuracy of Estimator
% figure(1)
% hold on; grid on;
% plot(real_value.Time,real_value.Data(:,1),LineWidth=2);
% plot(est_value.Time,est_value.Data(:,1),LineWidth=2)
% legend(["z","zhat"]);
% title("Cart")
% xlabel("Time [s]")
% ylabel("Displacement [m]")
% 
% figure(2)
% hold on; grid on;, 
% plot(real_value.Time,real_value.Data(:,2),LineWidth=2);
% plot(est_value.Time,est_value.Data(:,2),LineWidth=2)
% legend(["dz","dzhat"])
% title("Cart")
% xlabel("Time [s]")
% ylabel("Velocity [m/s]")
% 
% figure(3)
% hold on; grid on;
% plot(real_value.Time,real_value.Data(:,3),LineWidth=2);
% plot(est_value.Time,est_value.Data(:,3),LineWidth=2)
% legend(["theta","theta_hat"])
% title("Pole")
% xlabel("Time [s]")
% ylabel("Angle [rad]")
% 
% figure(4)
% hold on; grid on;
% plot(real_value.Time,real_value.Data(:,4),LineWidth=2);
% plot(est_value.Time,est_value.Data(:,4),LineWidth=2)
% legend(["dtheta","dthetahat"])
% title("Pole")
% xlabel("Time [s]")
% ylabel("Angular Velocity [m/s]")
% legend("theta","dtheta")
% title("Pole")
% 
% figure(5)
% hold on; grid on;
% plot(control_input1,LineWidth = 2);
% title("Control Input")
% xlabel("Time [s]")
% ylabel("Control Input [N]")

%% plot results
close all;
%% 基本設定
% PLOT for Simulink Result
set(0,'defaultAxesFontSize',14)
set(0,'defaultTextFontSize',14)
set(0,'defaultAxesFontName','Times New Roman')
set(0,'defaultTextFontName','Times New Roman')
% set(0,'defaultAxesFontName','Arial')
% set(0,'defaultTextFontName','Arial')
set(0,'defaultAxesXColor',[0 0 0])
set(0,'defaultAxesYColor',[0 0 0])
set(0,'defaultAxesZColor',[0 0 0])
% 凡例の位置 bestとかnorthoutsideとか
set(groot, 'defaultLegendLocation', 'best')
% 凡例の縦横
set(groot, 'defaultLegendOrientation', 'vertical')
tate = 200;
yoko = 400;
%% フォルダ作成
dd = datestr(now,'yyyy_mmdd_HHMMSS'); %日付・時刻の取得
if not(exist("results",'dir'))
    mkdir("results");
end
if savefig
mkdir('./results',dd); % resultsのフォルダに'yyyy_mmdd_HHMM'という名前のフォルダを生成
dd = strcat('./results/',dd);
end
%% シミュレーション動画作成
if savefig
mov_name = dd+"/landing_movie";
smwritevideo("cartpole",mov_name,'PlaybackSpeedRatio', 1.0, 'FrameRate', 60, 'VideoFormat', 'mpeg-4');
end
%% draw figures
figure('Name','States','Position',[500,100,yoko,tate+20])
grid on; hold on;
plot(output.Time,output.Data,"Linewidth",1.5)
legend(["z","theta","dz","dtheta"])
if savefig
saveas(gcf,strcat(dd,'/states')) ;
end

figure('Name','Control input','Position',[500,100,yoko,tate+20])
grid on; hold on;
plot(control_input.Time,control_input.Data,"Linewidth",1.5)
if savefig
saveas(gcf,strcat(dd,'/control_input')) ;
end

figure('Name','Control input','Position',[500,100,yoko,tate+20])
grid on; hold on;
plot(error.Time,error.Data,"Linewidth",1.5)
legend(["z","theta","dz","dtheta"])
if savefig
saveas(gcf,strcat(dd,'/error')) ;
end