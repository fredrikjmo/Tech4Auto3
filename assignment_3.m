clc; clear all;

%% Initialization of constants
C_f = 2*60000;
C_r = 2*57000;
m_v = 1575;
l_r = 1.5;
l_f = 1.3;
J_psi = 2875;

delta_constraint = deg2rad(25);

V_x = 20; % [m/s]

%% State space model definition

A = [0                1                             0                                 0                   ;
     0       -(C_f + C_r)/(m_v*V_x)           (C_f + C_r)/m_v            (C_r*l_r - C_f*l_f)/(m_v*V_x)    ;
     0                0                             0                                 1                   ;
     0  (C_r*l_r - C_f*l_f)/(J_psi*V_x)   (C_f*l_f - C_r*l_r)/J_psi  -(C_r*l_r^2 + C_f*l_f^2)/(J_psi*V_x)];

B_1 = [0; C_f/m_v; 0; (C_f*l_f)/J_psi];

B_2 = [0; (C_r*l_r - C_f*l_f)/(m_v*V_x) - V_x; 0; -(C_r*l_r^2 + C_f*l_f^2)/(J_psi*V_x)];

C = eye(4);

%% Feedback and feedforward control matrices

poles = [-1+1i -1-1i -1.5 -0.5];

K = place(A,B_1, poles);

K_ff = (m_v*V_x^2/l)*(l_r/C_f-l_f/C_r+l_f/C_r*K(3))+l-l_r*K(3);