% Cell of Question 2.A
% Preliminary Step: Verifying inner loop
clear all; close all; clc;

%% Given Variables
Z_alpha = -1231.914;
M_alpha = -299.26; % Uncertain Parameter #1
M_q = 0;
Z_delta = -107.676;
M_delta = -130.866; % Uncertain Parameter #2
A_alpha = -1429.131;
A_delta = -114.159;
V = 947.684;
g = 9.81;
w_alpha = 150;
zeta_alpha = 0.7;
r_M_alpha=57.813;
r_M_delta=32.716;

% Uncertain parameters
M_alpha=ureal('M_alpha',-299.26,'Percentage',[-r_M_alpha, +r_M_alpha]);
M_delta=ureal('M_delta',-130.866,'Percentage',[-r_M_delta, +r_M_delta]);

% Matrices of Actuator Model
A_act = [0, 1; -w_alpha^2, -2*zeta_alpha*w_alpha]; % A matrix of actuator model
B_act = [0; w_alpha^2]; % B matrix of actuator model
C_act = [1, 0]; % C matrix of actuator model
D_act = 0; % D matrix of actuator model

% State Space of Actuator Model
Gs_act = ss(A_act, B_act, C_act, D_act, 'StateName', {'\delta_q', '\delta_q_dot'}, 'InputName', {'\delta_q_c'}, 'OutputName', {'\delta_q'});

% Matrices of Airframe
A_af_u = [zeta_alpha/V, 1; M_alpha, M_q]; % A matrix of airframe
B_af_u = [Z_delta/V; M_delta]; % B matrix of airframe
C_af_u = [A_alpha/g, 0; 0, 1]'; % C matrix of airfram
D_af_u = [A_delta/g, 0]'; % D matrix of airframe

% State Space of Airframe
Gs_af_u = ss(A_af_u, B_af_u, C_af_u, D_af_u, 'StateName', {'alpha', 'q'}, 'InputName', {'\delta_q'}, 'OutputName', {'a_z', 'q'});

% Matrices of Sensor
A_s = zeros(2); % A matrix of sensor
B_s = zeros(2); % B matrix of sensor
C_s = zeros(2); % C matrix of sensor
D_s = eye(2); % D matrix of sensor

% State Space of Sensor
Gs_s = ss(A_s, B_s, C_s, D_s, 'StateName', {'alpha', 'q'}, 'InputName', {'a_z', 'q'}, 'OutputName', {'a_z_m', 'q_m'});

% Inner Loop Gain Kq
Kq = tunableGain('Kq',1,1);
Kq.Gain.Value = -0.165;
Kq.InputName = 'e_q'; 
Kq.OutputName = '\delta_q_c';

% Sum of Junctions
Sum = sumblk('e_q = q_c - q_m');

% Open Loop System
sys_open = connect(Gs_act,Gs_af_u,Gs_s,'\delta_q_c',{'a_z_m','q_m'});

%% Inner Loop System
sys_inner = connect(Gs_act,Gs_af_u,Gs_s,Kq,Sum,'q_c',{'a_z_m','q_m'});

% Outer Loop Gain
Ksc = tunableGain('Ksc',1,1);
steady_state_gain = 1/dcgain(sys_inner);
Ksc.Gain.Value = steady_state_gain(1); 
Ksc.Gain.Value = 2.3761; 
Ksc.InputName = 'q_s_c';
Ksc.OutputName = 'q_c';

% Conncet the models
sys_outter = connect(Gs_act,Gs_af_u,Gs_s,Kq,Ksc,Sum,'q_s_c',{'a_z_m','q_m'});

%% Analysis of Inner Loop

% Transferfunction of inner loop
inner_tf=tf(sys_inner)

% Information of Step Response: settling time and overshoot
inner_step = stepinfo(sys_inner);

% 2%
settlingTime = inner_step.SettlingTime;

% Percentage overshoot, relative to final y
overshot = inner_step.Overshoot;
 
%% Plotting

figure;
step(ss(sys_inner));
title("Inner Loop Step Response of Nominal System");











