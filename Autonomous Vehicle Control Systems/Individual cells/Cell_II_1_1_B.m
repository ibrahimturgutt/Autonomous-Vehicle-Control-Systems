% Cell of Question 1.1.B
% LFT Model
clear all; close all; clc;

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






