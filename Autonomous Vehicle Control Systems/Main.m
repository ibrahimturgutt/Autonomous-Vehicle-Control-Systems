clear all;, close all;, clc;

%% Cell of Question 1.1.A
% Creating System Model of Nominal and Uncertain Open Loop System

% Given Variables
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


% Nominal Model

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

% Uncertain Model

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

% Plots of Nominal Model

figure;
iopzplot(sys(1));
title("Z Plane of Nominal System a_z");

figure;
iopzplot(sys(2));
title("Z Plane of Nominal System q");

% Plots of Uncertain Model

figure;
iopzplot(sys2(1));
title("Z Plane of Uncertain System a_z");

figure;
iopzplot(sys2(2));
title("Z Plane of Uncertain System q");



%% Cell of Question 1.1.B
% LFT Model

% Given Variables
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

% Uncertain parameters
M_alpha = ureal('M_alpha',-299.26,'Percentage',[-r_M_alpha, +r_M_alpha]);
M_delta = ureal('M_delta',-130.866,'Percentage',[-r_M_delta, +r_M_delta]);

% Matrices of Airframe
A_af_u = [Z_alpha/V, 1; M_alpha, M_q]; % A matrix of airframe
B_af_u = [Z_delta/V; M_delta]; % B matrix of airframe
C_af_u = [A_alpha/g, 0 ; 0, 1 ]'; % C matrix of airframe
D_af_u = [A_delta/g, 0]'; % D matrix of airframe

% State Space of Airframe
Gs_af_u = ss(A_af_u,B_af_u,C_af_u,D_af_u,'StateName',{'alpha','q'},'InputName',{'\delta_q'},'OutputName',{'a_z','q'});

% Decomposition of Uncertain Object
[M, delta] = lftdata(Gs_af_u);

% LFT SS System
LFT = ss(M);

% Simplyfying
ssLFT = simplify(LFT,'full')

% Vlidation Step: Expected Matrices
A_exp= [ (Z_alpha/V), 1; (M_alpha.NominalValue), (M_q)];
B_exp= [0, 0, (Z_delta/V); sqrt(-M_alpha.NominalValue*(r_M_alpha/100)), sqrt(-M_delta.NominalValue*(r_M_delta/100)), M_delta.NominalValue];
C_exp= [sqrt(-M_alpha.NominalValue*(r_M_alpha/100)), 0; 0, 0; (A_alpha/g), 0; 0, 1];
D_exp= [0, 0, 0; 0, 0, sqrt(-M_delta.NominalValue*(r_M_delta/100)); 0, 0, (A_delta/g); 0, 0, 0];


% Comparison
C1 = isequal(A_exp, ssLFT.A);
C2 = isequal(B_exp, ssLFT.B);
C3 = isequal(C_exp, ssLFT.C);
C4 = isequal(D_exp, ssLFT.D);
if [C1, C2, C3, C4] == [1, 1, 1, 1]
    fprintf("Succesful Validation")
else
    fprintf("Validation Failed")
end



%% Cell of Question 1.2.A
% Uncertain inner loop state space

% Given Variables
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
Gs_act = ss(A_act,B_act,C_act,D_act,'StateName',{'\delta_q','\delta_q_dot'},'InputName',{'\delta_q_c'},'OutputName',{'\delta_q'});

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

% Sum of Junctions
Sum = sumblk('e_q = q_c - q_m');

% Open Loop System
sys_open = ss(connect(Gs_act,Gs_af_u,Gs_s,'\delta_q_c',{'a_z_m','q_m'}));

% Inner Loop

% Inner Loop Gain Kq
Kq = tunableGain('Kq',1,1);
Kq.Gain.Value = -0.25;
Kq.InputName = 'e_q'; 
Kq.OutputName = '\delta_q_c';

% Total System
sys_inner = connect(Gs_act,Gs_af_u,Gs_s,Kq,Sum,'q_c',{'a_z_m','q_m'});

% Outer Loop
% Outer Loop Gain
Ksc = tunableGain('Ksc',1,1);
steady_state_gain = 1/dcgain(sys_inner);
Ksc.Gain.Value = steady_state_gain(1); 
Ksc.InputName = 'q_s_c';
Ksc.OutputName = 'q_c';

% define K_sc for unitary steady state gain
Ksc_sim=tf(1/(-7.9849));  %gain found via bodeplot
Ksc_sim.Inputname = 'q_s_c'; 
Ksc_sim.OutputName = 'q_c';

% Total System
sys_outter = connect(Gs_act,Gs_af_u,Gs_s,Kq,Ksc,Sum,'q_s_c',{'a_z_m','q_m'});

% Plotting

figure;
rlocus(sys_inner(1));
sgrid(zeta,20000);
title("Inner Loop Root Locus of Uncertain System");

figure;
rlocus(ss(sys_inner(1)));
sgrid(zeta,20000);
title("Inner Loop Root Locus of Nominal System");

figure;
step(sys_inner);
title("Inner Loop Step Response of Uncertain System");

figure;
step(ss(sys_inner));
title("Inner Loop Step Response of Nominal System");



%% Cell of Question 1.2.B
% Uncertainty weight computation

% Given Variables
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

% Additive
% Fits an uncertain model to a family of LTI models with unstructured uncertainty
[P1,InfoS12] = ucover(p_array_a_z,nom_sys(1),1,'Additive');
[P2,InfoS22] = ucover(p_array_a_z,nom_sys(1),2,'Additive');
[P3,InfoS32] = ucover(p_array_a_z,nom_sys(1),3,'Additive');
[P4,InfoS42] = ucover(p_array_a_z,nom_sys(1),4,'Additive');

% Fits an uncertain model to a family of LTI models with unstructured uncertainty
[P1,InfoS12] = ucover(p_array_a_z1,nom_sys(1),2,'Additive');
[P2,InfoS22] = ucover(p_array_a_z2,nom_sys(1),2,'Additive');
[P3,InfoS32] = ucover(p_array_a_z3,nom_sys(1),2,'Additive');
[P4,InfoS42] = ucover(p_array_a_z4,nom_sys(1),2,'Additive');

% Plotting

% Bode plot weigthed tf
Gp = ss(p_array_a_z);
G = nom_sys(1);
relerr = (Gp-G)/G;

figure;
bodemag(relerr,'b',InfoS1.W1,'r-.',InfoS2.W1,'k-.',InfoS3.W1,'m-.',InfoS4.W1,'g-.',{0.1,1000});
legend('(G_p-G)/G)','Lm(\omega)First-order w','Lm(\omega)Second-order w','Lm(\omega)Third-order w','Lm(\omega)Fourth-order w','Location','SouthWest');
title("Singular Values");



%% Cell of Question 2.A
% Preliminary Step: Verifying inner loop

% Given Variables
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

% Inner Loop System
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

% Analysis of Inner Loop

% Transferfunction of inner loop
inner_tf=tf(sys_inner)

% Information of Step Response: settling time and overshoot
inner_step = stepinfo(sys_inner);

% %2
settlingTime = inner_step.SettlingTime;

% Percentage overshoot, relative to yfinal
overshot = inner_step.Overshoot;

% Plotting

figure;
step(ss(sys_inner));
title("Inner Loop Step Response of Nominal System");



%% Cell of Question 2.B
% Reference Computation Model

% Given Variables
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
r_M_alpha = 57.813;
r_M_delta = 32.716;

% Uncertain parameters
M_alpha = ureal('M_alpha',-299.26,'Percentage',[-r_M_alpha, +r_M_alpha]);
M_delta = ureal('M_delta',-130.866,'Percentage',[-r_M_delta, +r_M_delta]);

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

% Computation of Reference model
% Transfer Function Computing
% Maximum of 2% overshoot
zeta_rm = (-log(0.02))/(sqrt(pi^2+log(0.02)^2))+0.009;

% 0.2 seconcs
w_rm = -log(0.02*sqrt(1-zeta_rm^2))/(zeta_rm*0.2)-7.6;
inner_zeros = zero(sys_inner(1));
z_n_m_p = inner_zeros(1);

% Define transfer function
T_rm = tf([((-w_rm^2)/z_n_m_p) w_rm^2],[1 2*zeta_rm*w_rm w_rm^2]);

% Plotting
step_information = stepinfo(T_rm)
figure;
step(T_rm);



%% Cell of Question 2.C
% Weighting filter selection

% Given Variables
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

% Parameter Computing
% Maximum 2% overshoot
zeta_rm = (-log(0.02))/(sqrt(pi^2+log(0.02)^2))+0.009;

% 0.2 seconcs
w_rm = -log(0.02*sqrt(1-zeta_rm^2))/(zeta_rm*0.2)-7.6;

% Define transfer function
T_rm = tf([((-w_rm^2)/z_n_m_p) w_rm^2],[1 2*zeta_rm*w_rm w_rm^2]);

% Sensitivity function Analysis
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

% Complementary sensitivity function Analysis
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

% Plotting

figure;
sigma(s_t,'r-',T_rm,'c-',1/W1,'k-',1/W2,'g-');
title("Singular Values of Weight Filter");



%% Cell of Question 2.D
% Synthesis block diagram

% Given Variables
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

% computation of the non minimum phase zero of 
inner_zeros = zero(sys_inner(1));
z_n_m_p = inner_zeros(1);

% Outer Loop Gain
Ksc = tunableGain('Ksc',1,1);
steady_state_gain = 1/dcgain(sys_inner);
Ksc.Gain.Value = steady_state_gain(1); 
Ksc.InputName = 'q_s_c';
Ksc.OutputName = 'q_c';

% Outer Loop System
sys_outer = connect(Gs_act,Gs_af_u,Gs_s,Kq,Ksc,Sum,'q_s_c',{'a_z_m','q_m'});

% Maximum 2% overshoot
zeta_rm = (-log(0.02))/(sqrt(pi^2+log(0.02)^2))+0.009;

% 0.2 seconcs
w_rm = -log(0.02*sqrt(1-zeta_rm^2))/(zeta_rm*0.2)-7.6;

% Define transfer function
T_rm = tf([((-w_rm^2)/z_n_m_p) w_rm^2],[1 2*zeta_rm*w_rm w_rm^2]);
T_rm.InputName = {'a_z_c'}; 
T_rm.OutputName = {'a_z_t'};

% Analysis of sensitivity function
s_t = 1-T_rm;
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
w_t = tf(T_rm);

% Gain respones of the system
[mag_t,phase,wout] = bode(T_rm);

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

% mixsyn shapes the singular values of the sensitivity function S
[K,CL,GAM] = mixsyn(T_rm,W1,W2,W3);

% Summing junctions
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
Kz.InputName = {'a_z_tmd','q_s_c'}; 
Kz.OutputName = {'z_1_tilda','z_2_tilda'};

Kv = tunableGain('K_v',2,2);
Kv.Gain.Value = eye(2);
Kv.InputName = {'a_z_m_d','a_z_c'}; 
Kv.OutputName = {'v_1','v_2'};

% Computing the orange box transfer function
P_s_tilda = connect(sys_outer,T_rm,Kz,Kv,Sum_a_in,Sum_a_rm,sum_d,sum_r,sum_u,{'r','d','u'},{'z_1_tilda','z_2_tilda','v_1','v_2'});

% Tf
Tf_p_s = tf(P_s_tilda);

% Computing the transfer function of the orange + green box

% Weight matrices of component #1
Tf_ws1 = tf(W1);
Tf_ws1.InputName ='z_1_tilda';
Tf_ws1.OutputName ='z_1';

% Weight matrices of component #2
Tf_ws2 = tf(W2);
Tf_ws2.InputName ='z_2_tilda';
Tf_ws2.OutputName ='z_2';

% Connceted system
P_s = connect(P_s_tilda,Tf_ws1,Tf_ws2,{'r','d','u'},{'z_1','z_2','v_1','v_2'});

% Final Tf
tf_tilda = tf(P_s_tilda)
tf_full = tf(P_s)

% Plotting

figure;
iopzplot(tf_full);
title("Z-Plane Diagram of Orange + Green Box");

figure;
iopzplot(tf_tilda);
title("Z-Plane Diagram of Orange Box");



%% Cell of Question 2.E
% Controller Synthesis

% Given Variables
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

% Paraemter for simulink model
Kq_value=-0.165;

% Sum of Junctions
Sum = sumblk('e_q = q_c - q_m');

% Inner Loop System
sys_inner = connect(Gs_act,Gs_af_u,Gs_s,Kq,Sum,'q_c',{'a_z_m','q_m'});

% Reference model computation:

% Computation of the non minimum phase zero  
inner_zeros = zero(sys_inner(1));
z_n_m_p = inner_zeros(1);

% Outer Loop Gain
Ksc = tunableGain('Ksc',1,1);
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
T_rm = tf([((-w_rm^2)/z_n_m_p) w_rm^2],[1 2*zeta_rm*w_rm w_rm^2]);
T_rm.InputName = {'a_z_c'}; 
T_rm.OutputName = {'a_z_t'};

% Analysis of sensitivity function
s_t = 1-T_rm;
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
w_t = tf(T_rm);

% Gain respones of the system
[mag_t,phase,wout] = bode(T_rm);

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
Kz.OutputName = {'z_1_tilda','z_2_tilda'};

Kv = tunableGain('K_v',2,2);
Kv.Gain.Value = eye(2);
Kv.InputName = {'a_z_m_d','a_z_c'}; 
Kv.OutputName = {'v_1','v_2'};

% Computing the orange box transfer function
P_s_tilda= connect(sys_outer,T_rm,Kz,Kv,Sum_a_in,Sum_a_rm,sum_d,sum_r,sum_u,{'r','d','u'},{'z_1_tilda','z_2_tilda','v_1','v_2'});

% Computing the transfer function of the orange + green box

% Weight matrices of component #1
Tf_ws_1 = tf(W1);
Tf_ws_1.InputName = 'z_1_tilda';
Tf_ws_1.OutputName = 'z_1';

% Weight matrices of component #2
Tf_ws_2 = tf(W2);
Tf_ws_2.InputName ='z_2_tilda';
Tf_ws_2.OutputName ='z_2';

% Connceted system
P_s = connect(P_s_tilda,Tf_ws_1,Tf_ws_2,{'r','d','u'},{'z_1','z_2','v_1','v_2'});

% Augmented Plant for H-Infinity Synthesis

% Total number of controller inputs
NMEAS = 2;

% Total number of controller outputs
NCON = 1;

% H-INF Structure
[K_F0,CL,GAM] = hinfsyn(P_s,NMEAS,NCON); 

figure;
bode(K_F0(2),'m-',CL,'g-');

% Obtaining the decomposed control system gains
K_dr = K_F0(1);
K_cf = K_F0(2);

% Minimal control orders
ordDr = tf(minreal(K_dr));
ordCf = tf(minreal(K_cf));

% Weighted closed loop transfer matrix components
T_r_z1 = tf(P_s(1,1));
T_d_z1 = tf(P_s(1,2));
T_r_z2 = tf(P_s(2,1));
T_d_z2 = tf(P_s(2,2));

% Unweighted closed loop transfer matrix components
UT_r_z1 = tf(P_s_tilda(1,1));
UT_d_z1 = tf(P_s_tilda(1,2));
UT_r_z2 = tf(P_s_tilda(2,1));
UT_d_z2 = tf(P_s_tilda(2,2));

% Performance level 
perf_level = hinfnorm(CL);

% Plotting

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



%% Cell of Question 2.F
% Controller Implementation

% Given Variables
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
Ksc = tunableGain('Ksc',1,1);
steady_state_gain = 1/dcgain(sys_inner);
Ksc.Gain.Value = steady_state_gain(1); 
Ksc.InputName = 'q_s_c';
Ksc.OutputName = 'q_c';

% Outer Loop System
sys_outer = connect(Gs_act,Gs_af_u,Gs_s,Kq,Ksc,Sum,'q_s_c',{'a_z_m','q_m'});

% Maximum of 2% overshoot
zeta_rm = (-log(0.02))/(sqrt(pi^2+log(0.02)^2))+0.009;

%  0.2 seconds
w_rm = -log(0.02*sqrt(1-zeta_rm^2))/(zeta_rm*0.2)-7.6;

% Define transfer function
T_rm = tf([((-w_rm^2)/z_n_m_p) w_rm^2],[1 2*zeta_rm*w_rm w_rm^2]);
T_rm.InputName = {'a_z_c'}; 
T_rm.OutputName = {'a_z_t'};

% Analysis of sensitivity function
s_t = 1-T_rm;
w_s = 1/s_t;

% Getting gain respones of the system
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
w_t = tf(T_rm);

% Getting gain respones of the system
[mag_t,phase,wout] = bode(T_rm);

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

% mixsyn shapes the singular values of the sensitivity function S
[K,CL,GAM] = mixsyn(T_rm,W1,W2,W3);

% Block Diagram Synthesis

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
Kz.OutputName = {'z_1_tilda','z_2_tilda'};

Kv = tunableGain('K_v',2,2);
Kv.Gain.Value = eye(2);
Kv.InputName = {'a_z_m_d','a_z_c'}; 
Kv.OutputName = {'v_1','v_2'};

% Computing the orange box transfer function
P_s_tilda= connect(sys_outer,T_rm,Kz,Kv,Sum_a_in,Sum_a_rm,sum_d,sum_r,sum_u,{'r','d','u'},{'z_1_tilda','z_2_tilda','v_1','v_2'});

% Computing the transfer function of the orange + green box

% Weight matrices of component #1
Tf_ws_1 = tf(W1);
Tf_ws_1.InputName = 'z_1_tilda';
Tf_ws_1.OutputName = 'z_1';

% Weight matrices of component #2
Tf_ws_2 = tf(W2);
Tf_ws_2.InputName ='z_2_tilda';
Tf_ws_2.OutputName ='z_2';

% Connceted system
P_s = connect(P_s_tilda,Tf_ws_1,Tf_ws_2,{'r','d','u'},{'z_1','z_2','v_1','v_2'});

% Augmented Plant for H-Infinity Synthesis

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
K_dr = tf(minreal(K_dr));
K_cf = tf(minreal(K_cf));

% Obtaining converted controllers
K_cf_prime = K_cf/K_dr;
K_dr_prime = K_dr;

% Controller analysis

% Order of the controller
order_K_cf_prime = order(K_cf_prime)
order_K_df_prime = order(K_dr_prime)

% Reduction of controller
K_cf_prime_reduced = reduce(K_cf_prime,3);
K_dr_prime_reduced = reduce(K_dr_prime,3);

% Order of the reduced controller
reduced_order_K_cf = order(K_cf_prime_reduced)
reduced_order_K_dr = order(K_dr_prime_reduced)

% Ploting

figure;
iopzplot(K_cf_prime_reduced);
title("Zeros and Poles of K_(cf)'_(red)'");

figure;
iopzplot(K_dr_prime_reduced);
title("Zeros and Poles of K_(dr)'_(red)'");

figure;
iopzplot(K_cf_prime);
title("Zeros and Poles of K_(cf)'");

figure;
iopzplot(K_dr_prime);
title("Zeros and Poles - K_(dr)'");

figure;
sigma(K_cf_prime_reduced,'r-',K_dr_prime,'c-',K_cf_prime,'k-',K_dr_prime,'g-')
legend('K_cf_prime-reduced','K_dr_prime-reduced','K_cf_prime','K_dr_prime','Location','Northwest')
title("Simngular Values - Full and Reduced Order");

