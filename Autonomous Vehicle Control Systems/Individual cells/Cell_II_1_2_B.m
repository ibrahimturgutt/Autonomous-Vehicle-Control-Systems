% Cell of Question 1.2.B
% Uncertainty weight computation
clear all; close all; clc;
run("Cell_II_1_2_A.m");

%% Given Variables
Z_alpha = -1231.914;
M_alpha = -299.26; % Uncertain Parameter #1
M_q = 0;
Z_delta = -107.676;
M_delta = -130.866; % Uncertain Parameter #2
A_alpha = -1429.131;
A_delta = -114.59;
V = 947.684;
g = 9.81;
w_a = 150;
zeta_a = 0.7;
r_M_alpha = 57.813;
r_M_delta = 32.716;
zeta = 0.707;

% Uncertain parameters
M_alpha = ureal('M_alpha',-299.26,'Percentage',[-r_M_alpha, +r_M_alpha]);
M_delta = ureal('M_delta',-130.866,'Percentage',[-r_M_delta, +r_M_delta]);

% Matrices of Actutor Model
A_act = [0, 1 ; -w_a^2, -2*zeta_a*w_a]; % A matrix of actuator model
B_act = [0; w_a^2]; % B matrix of actuator model
C_act = [1, 0]; % C matrix of actuator model
D_act = 0; % D matrix of actuator model

% State Space of Actuator Model
Gs_act = ss(A_act,B_act,C_act,D_act,'StateName',{'\delta_q','\delta_q_dot'},'InputName',{'\delta_q_c'},'OutputName',{'\delta_q'})

% Matrices of Airframe
A_af_u = [Z_alpha/V, 1; M_alpha, M_q]; % A matrix of airframe
B_af_u = [Z_delta/V; M_delta]; % B matrix of airframe
C_af_u = [A_alpha/g, 0 ; 0, 1 ]'; % C matrix of airframe
D_af_u = [A_delta/g, 0]'; % D matrix of airframe

% State Space of Airframe
Gs_af_u = ss(A_af_u,B_af_u,C_af_u,D_af_u,'StateName',{'alpha','q'},'InputName',{'\delta_q'},'OutputName',{'a_z','q'});

% Matrices of Sensor
A_s = zeros(2); % A matrix of sensor
B_s = zeros(2); % B matrix of sensor
C_s = zeros(2); % C matrix of sensor
D_s = eye(2); % D matrix of sensor

% State Space of Sensor
Gs_s = ss(A_s,B_s,C_s,D_s,'StateName',{'alpha','q'},'InputName',{'a_z','q'},'OutputName',{'a_z_m','q_m'});

% Inner Loop Gain Kq
Kq = tunableGain('Kq',1,1);
Kq.Gain.Value = -0.25;
Kq.InputName = 'e_q'; 
Kq.OutputName = '\delta_q_c';

% Sum of Junctions
Sum = sumblk('e_q = q_c - q_m');

% Outer Loop Gain
Ksc = tunableGain('Ksc',1,1);
steady_state_gain = 1/dcgain(sys_inner);
Ksc.Gain.Value = steady_state_gain(1); 
Ksc.InputName = 'q_s_c';
Ksc.OutputName = 'q_c';


% Total System
sys_outter = connect(Gs_act,Gs_af_u,Gs_s,Kq,Ksc,Sum,'q_s_c',{'a_z_m','q_m'});

% Decomposition of Uncertain Objects
[M,Delta] = lftdata(sys_outter)

% Samples of uncertain state space system
p_array = usample(sys_outter,50);
p_array_a_z = usample(sys_outter(1),50);
nom_sys=ss(sys_outter);

% Samples of uncertain state space system
p_array_a_z1 = usample(sys_outter(1),10);
p_array_a_z2 = usample(sys_outter(1),20);
p_array_a_z3 = usample(sys_outter(1),30);
p_array_a_z4 = usample(sys_outter(1),40);

% Fits an uncertain model to a family of LTI models with unstructured uncertainty
[P1,InfoS1] = ucover(p_array_a_z1,nom_sys(1),2,'InputMult');
[P2,InfoS2] = ucover(p_array_a_z2,nom_sys(1),2,'InputMult');
[P3,InfoS3] = ucover(p_array_a_z3,nom_sys(1),2,'InputMult');
[P4,InfoS4] = ucover(p_array_a_z4,nom_sys(1),2,'InputMult');


nom_sys1=ss(sys_outter);

% Fits an uncertain model to a family of LTI models with unstructured uncertainty
[P11,InfoS11] = ucover(p_array_a_z1,nom_sys1(1),2,'InputMult');
[P21,InfoS21] = ucover(p_array_a_z2,nom_sys1(1),2,'InputMult');
[P31,InfoS31] = ucover(p_array_a_z3,nom_sys1(1),2,'InputMult');
[P41,InfoS41] = ucover(p_array_a_z4,nom_sys1(1),2,'InputMult');

% LTI GAIN system for simulink
Kq_sim = tf(-0.165); % -0.165 for q_m found via rlocus
Kq_sim.Inputname = 'e_q'; 
Kq_sim.OutputName = '\delta_q_c';

%% Plotting

Gp = ss(p_array_a_z);
G = nom_sys(1);
relerr = (Gp-G)/G;

figure;
bodemag(relerr,'b',InfoS1.W1,'r-.',InfoS2.W1,'k-.',InfoS3.W1,'m-.',InfoS4.W1,'g-.',{0.1,1000});
legend('(G_p-G)/G)','Lm(\omega)First-order w','Lm(\omega)Second-order w','Lm(\omega)Third-order w','Lm(\omega)Fourth-order w','Location','SouthWest');
title("Singular Values");








