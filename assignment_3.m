clc; clear all;

%% Initialization of constants
C_f = 2*60000;
C_r = 2*57000;
m_v = 1575;
l_r = 1.5;
l_f = 1.3;
l = l_f + l_r;
J_psi = 2875;

delta_constraint = 25;

V_x = 10; % [m/s]

a_11 = 0;
a_12 = 1;
a_13 = 0;
a_14 = 0;

a_21 = 0;
a_22 = -(C_f + C_r)/(m_v*V_x);
a_23 = (C_f + C_r)/m_v;
a_24 = (C_r*l_r - C_f*l_f)/(m_v*V_x);

a_31 = 0;
a_32 = 0;
a_33 = 0;
a_34 = 1;

a_41 = 0;
a_42 = (C_r*l_r - C_f*l_f)/(J_psi*V_x);
a_43 = (C_f*l_f - C_r*l_r)/J_psi;
a_44 = -(C_r*l_r^2 + C_f*l_f^2)/(J_psi*V_x);

b1_1 = 0;
b1_2 = C_f/m_v;
b1_3 = 0;
b1_4 = (C_f*l_f)/J_psi;

b2_1 = 0;
b2_2 = (C_r*l_r - C_f*l_f)/(m_v*V_x) - V_x;
b2_3 = 0;
b2_4 = -(C_r*l_r^2 + C_f*l_f^2)/(J_psi*V_x);


%% State space model definition

A = [a_11 a_12 a_13 a_14;
     a_21 a_22 a_23 a_24;
     a_31 a_32 a_33 a_34;
     a_41 a_42 a_43 a_44];

B_1 = [b1_1; b1_2; b1_3; b1_4];
B_2 = [b2_1; b2_2; b2_3; b2_4];

B = [B_1 B_2];

C = eye(4);

%% LQR Feedback and feedforward control matrices

q_11 = 100;
q_22 = 1;
q_33 = 1;
q_44 = 1;

Q = [q_11 0 0 0;
     0 q_22 0 0;
     0 0 q_33 0;
     0 0 0 q_44];

R = 1;

[K,S,P] = lqr(A,B_1,Q,R);

K_ff = (m_v*V_x^2/l)*(l_r/C_f-l_f/C_r+(l_f/C_r)*K(3))+l-l_r*K(3);


