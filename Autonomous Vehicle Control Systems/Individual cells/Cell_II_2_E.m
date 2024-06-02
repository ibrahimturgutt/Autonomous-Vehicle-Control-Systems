% Cell of Question 2.E
% Controller Synthesis
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

% Reference model computation:

% Computation of the non minimum phase zero 
inner_zeros = zero(sys_inner(1));
z_n_m_p = inner_zeros(1);

% Outer Loop Gain
Ksc = tunableGain('K_s_c',1,1);
steady_state_gain = 1/dcgain(sys_inner);
Ksc.Gain.Value = steady_state_gain(1); 
Ksc.InputName = 'q_s_c';
Ksc.OutputName = 'q_c';

% Outer Loop System
sys_outer = connect(Gs_act,Gs_af_u,Gs_s,Kq,Ksc,Sum,'q_s_c',{'a_z_m','q_m'});

% Maximum of 2% overshoot
zeta_rm = (-log(0.02))/(sqrt(pi^2+log(0.02)^2))+0.009;

% 0.2 seconcs
w_rm = -log(0.02*sqrt(1-zeta_rm^2))/(zeta_rm*0.2)-7.6;

% Define transfer function
T_r_m = tf([((-w_rm^2)/z_n_m_p) w_rm^2],[1 2*zeta_rm*w_rm w_rm^2]);
T_r_m.InputName = {'a_z_c'}; 
T_r_m.OutputName = {'a_z_t'};

% Analysis of sensitivity function
s_t = 1-T_r_m;
w_s = 1/s_t;

% Gain respones of the system
[mag_s,phase,wout] = bode(s_t);

% Computing the further needed parameter
w_i_s = 6.81; %from bode plot at gain -3 db
kis = 0.707;

% High and low frequency gain
lfgain_s = mag_s(1);
hfgain_s = mag_s(end);

% Needed pole
w_i_stroke_s = sqrt((hfgain_s^2-kis^2)/(kis^2-lfgain_s^2))*w_i_s;

% Analysis of complementary sensitivity function
w_t = tf(T_r_m);

% Gain respones of the system
[mag_t,phase,wout] = bode(T_r_m);

% Computing the further needed parameter
w_i_t = 20.8; %from bode plot at gain -3 db
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

% Values of the sensitivty function
S_o = (1/W1);
KS_o = (1/W2); 

% Junctions Sum
Sum_a_in = sumblk('a_z_m_d = a_z_m + a_z_d');
Sum_a_rm = sumblk('a_z_tmd = a_z_t- a_z_m_d');

% Change of input names
sum_r = sumblk('r = a_z_c');
sum_r.OutputName = {'a_z_c'}; 
sum_r.InputName = {'r'};

sum_d = sumblk('d = a_z_d');
sum_d.OutputName = {'a_z_d'}; 
sum_d.InputName = {'d'};

sum_u = sumblk('u = q_s_c');
sum_u.OutputName = {'q_s_c'}; 
sum_u.InputName = {'u'};

% Gain Matrices
Kz = tunableGain('K_z',2,2);
Kz.Gain.Value = eye(2);
Kz.InputName = {'a_z_tmd','a_z_d'}; 
Kz.OutputName = {'z_1_tild','z_2_tild'};

Kv = tunableGain('K_v',2,2);
Kv.Gain.Value = eye(2);
Kv.InputName = {'a_z_m_d','a_z_c'}; 
Kv.OutputName = {'v_1','v_2'};

% Computing the orange box transfer function
P_s_tild= connect(sys_outer,T_r_m,Kz,Kv,Sum_a_in,Sum_a_rm,sum_d,sum_r,sum_u,{'r','d','u'},{'z_1_tild','z_2_tild','v_1','v_2'});

% Computing the transfer function of the orange + green box

% Weight matrices of component #1
Tf_ws_1 = tf(W1);
Tf_ws_1.InputName = 'z_1_tild';
Tf_ws_1.OutputName = 'z_1';

% Weight matrices of component #2
Tf_ws_2 = tf(W2);
Tf_ws_2.InputName ='z_2_tild';
Tf_ws_2.OutputName ='z_2';

% Connceted system
P_s = connect(P_s_tild,Tf_ws_1,Tf_ws_2,{'r','d','u'},{'z_1','z_2','v_1','v_2'});

%% Augmented Plant for H-Infinity Synthesis

% Total number of controller inputs
NMEAS = 2;

% Total number of controller outputs
NCON = 1;

% H-INF Structure
[K_F0,CL,GAM] = hinfsyn(P_s,NMEAS,NCON); 

% Obtaining the decomposed control system gains
K_dr = K_F0(1);
K_cf = K_F0(2);

% Minimal control orders
tf_Dr = tf(minreal(K_dr))
tf_Cf = tf(minreal(K_cf))
order_Dr = order(tf_Dr)
order_Cf = order(tf_Cf)

% Weighted closed loop transfer matrix components
T_r_z1 = tf(P_s(1,1));
T_d_z1 = tf(P_s(1,2));
T_r_z2 = tf(P_s(2,1));
T_d_z2 = tf(P_s(2,2));

% Unweighted closed loop transfer matrix components
UT_r_z1 = tf(P_s_tild(1,1));
UT_d_z1 = tf(P_s_tild(1,2));
UT_r_z2 = tf(P_s_tild(2,1));
UT_d_z2 = tf(P_s_tild(2,2));

% Performance level 
perf_level = hinfnorm(CL);

%% Plotting

figure()
subplot(2,2,1)
sigma(UT_r_z1)
grid on
hold on
sigma(UT_r_z1*S_o)
sigma(UT_r_z1*KS_o)
hold off
title('Input: r  Output: z_(tilda)_1')

subplot(2,2,2)
sigma(UT_d_z1)
grid on
hold on
sigma(UT_d_z1*S_o)
sigma(UT_d_z1*KS_o)
hold off
title('Input: d  Output: z_(tilda)_1')

subplot(2,2,3)
sigma(UT_r_z2)
grid on
hold on
sigma(UT_r_z2*S_o)
sigma(UT_r_z2*KS_o)
hold off
title('Input: r  Output: z_(tilda)_2')

subplot(2,2,4)
sigma(UT_d_z2)
grid on
hold on
sigma(UT_d_z2*S_o)
sigma(UT_d_z2*KS_o)
hold off
title('Input: d  Output: z_(tilda)_2')





