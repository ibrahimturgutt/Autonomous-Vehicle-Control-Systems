% Cell of Question 2.2.C
% Weighting filter selection
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

% Inner Loop System
sys_inner = connect(Gs_act,Gs_af_u,Gs_s,Kq,Sum,'q_c',{'a_z_m','q_m'});

%% Reference model computation:

% Computation of the non minimum phase zero 
inner_zeros = zero(sys_inner(1));
z_n_m_p = inner_zeros(1);

%% Parameter Computing
% Maximum 2% overshoot
zeta_r_m = (-log(0.02))/(sqrt(pi^2+log(0.02)^2))+0.009;

% 0.2 seconcs
w_r_m = -log(0.02*sqrt(1-zeta_r_m^2))/(zeta_r_m*0.2)-7.6;

% Define transfer function
T_rm = tf([((-w_r_m^2)/z_n_m_p) w_r_m^2],[1 2*zeta_r_m*w_r_m w_r_m^2]);

%% Sensitivity function Analysis
s_t = 1-T_rm;
% Gain respones of system
[mag_s,phase,wout] = bode(s_t);

% Compute further needed parameter
w_i_s = 6.81; % from bode plot at gain -3 db
kis = 0.707;

% High and low frequency gain
lfgain_s = mag_s(1);
hfgain_s = mag_s(end);

% Needed pole
w_i_stroke_s = sqrt((hfgain_s^2-kis^2)/(kis^2-lfgain_s^2))*w_i_s;

%% Complementary sensitivity function Analysis
w_t = tf(T_rm);

% Gain respones of system
[mag_t,phase,wout] = bode(T_rm);

% Compute further needed parameter
w_i_t = 20.8; % again from bode plot at gain -3 db
kit = 0.707;

% High and low frequency gain
lfgain_t = mag_t(1);
hfgain_t = mag_t(end);

% Needed pole
w_i_stroke_t = sqrt((hfgain_t^2-kit^2)/(kit^2-lfgain_t^2))*w_i_t;
 
% Define weight
s = zpk('s');
W1 = inv((hfgain_s*s+w_i_stroke_s*lfgain_s)/(s+w_i_stroke_s));
W2 = inv((hfgain_t*s+w_i_stroke_t*lfgain_t)/(s+w_i_stroke_t));
W3 = [];

% Sensitivty function values
S_o=(1/W1);
KS_o=(1/W2); 

% mixsyn shapes the singular values of the sensitivity function S
[K,CL,GAM] = mixsyn(T_rm,W1,W2,W3);

%% Plotting

figure;
sigma(s_t,'r-',T_rm,'c-',1/W1,'k-',1/W2,'g-');
title("Singular Values of Weight Filter");