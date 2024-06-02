% Cell of Question 1.1.A
% Creating System Model of Nominal and Uncertain Open Loop System
clear all;, close all;, clc;

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


%% Nominal Model

% Matrices of Actutor Model
A_act = [0, 1 ; -w_a^2, -2*zeta_a*w_a]; % A matrix of actuator model
B_act = [0; w_a^2]; % B matrix of actuator model
C_act = [1, 0]; % C matrix of actuator model
D_act = 0; % D matrix of actuator model

% State Space of Actuator Model
Gs_act = ss(A_act,B_act,C_act,D_act,'StateName',{'\delta_q','\delta_q_dot'},'InputName',{'\delta_q_c'},'OutputName',{'\delta_q'})


% Matrices of Airframe
A_af_n = [Z_alpha/V, 1 ;M_alpha, M_q]; % A matrix of airframe
B_af_n = [Z_delta/V; M_delta]; % B matrix of airframe
C_af_n = [A_alpha/g, 0; 0, 1 ]'; % C matrix of airframe
D_af_n= [A_delta/g, 0]'; % D matrix of airframe

% State Space of Airframe
Gs_af_n = ss(A_af_n,B_af_n,C_af_n,D_af_n,'StateName',{'alpha','q'},'InputName',{'\delta_q'},'OutputName',{'a_z','q'});


% Matrices of Sensor
A_s = zeros(2); % A matrix of sensor
B_s = zeros(2); % B matrix of sensor
C_s = zeros(2); % C matrix of sensor
D_s = eye(2); % D matrix of sensor

% State Space of Sensor
Gs_s = ss(A_s,B_s,C_s,D_s,'StateName',{'alpha','q'},'InputName',{'a_z','q'},'OutputName',{'a_z_m','q_m'});

% Total system
sys = connect(Gs_act,Gs_af_n,Gs_s,'\delta_q_c',{'a_z_m','q_m'});

% Poles
p_n = pole(sys)

% Zeros
z_n = zero(sys)

% Damping and Eigenvalues of Nominal Model
eig(sys);
damp(sys);

%% Uncertain Model

% Uncertain parameters
M_alpha = ureal('M_alpha',-299.26,'Percentage',[-r_M_alpha, +r_M_alpha]);
M_delta = ureal('M_delta',-130.866,'Percentage',[-r_M_delta, +r_M_delta]);

% Only airframe depends on uncertain parameters. thus, we have new airframe matrices and state space.

% Matrices of Airframe
A_af_u = [Z_alpha/V, 1; M_alpha, M_q]; % A matrix of airframe
B_af_u = [Z_delta/V; M_delta]; % B matrix of airframe
C_af_u = [A_alpha/g, 0 ; 0, 1 ]'; % C matrix of airframe
D_af_u = [A_delta/g, 0]'; % D matrix of airframe

% State Space of Airframe
Gs_af_u = ss(A_af_u,B_af_u,C_af_u,D_af_u,'StateName',{'alpha','q'},'InputName',{'\delta_q'},'OutputName',{'a_z','q'});

% Total System
sys2 = connect(Gs_act,Gs_af_u,Gs_s,'\delta_q_c',{'a_z_m','q_m'});

% Damping and Eigenvalues of Uncertain Model
eig(sys2);
damp(sys2);

%% Plots of Nominal Model

figure;
iopzplot(sys(1));
title("Z Plane of Nominal System a_z");

figure;
iopzplot(sys(2));
title("Z Plane of Nominal System q");




%% Plots of Uncertain Model

figure;
iopzplot(sys2(1));
title("Z Plane of Uncertain System a_z");

figure;
iopzplot(sys2(2));
title("Z Plane of Uncertain System q");




