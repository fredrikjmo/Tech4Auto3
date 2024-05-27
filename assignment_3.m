clc; clear all;

%% Initialization of constants
C_f = 2*60000;
C_r = 2*57000;
m_v = 1575;
l_r = 1.5;
l_f = 1.3;
l = l_f + l_r;
J_psi = 2875;

delta_constraint = deg2rad(25);

V_x = 22.2; % [m/s] (~80 km/h)

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

C = eye(4);

%% LQR Feedback and feedforward control matrices

q_11 = 10;  % Constraint on lateral deviation
q_22 = 200; % Constraint on lateral deviation rate of change
q_33 = 10;  % Constraint on relative yaw deviation
q_44 = 300; % Constraint on relative yaw deviation rate of change

Q = [q_11 0 0 0;
     0 q_22 0 0;
     0 0 q_33 0;
     0 0 0 q_44];

R = 1; % Constraint on control signal (steering angle)

[K,S,P] = lqr(A,B_1,Q,R);

K_ff = (m_v*V_x^2/l)*(l_r/C_f-l_f/C_r+(l_f/C_r)*K(3))+l-l_r*K(3);


%% Plot LQR-gain as function of velocity

% Define your discrete values of x
x = 1:0.1:28; % Example values, change as needed

k1 = zeros(size(x));
k2 = zeros(size(x));
k3 = zeros(size(x));
k4 = zeros(size(x));

% Calculate values of k1, k2, k3, k4 for each x
for i = 1:length(x)
    ai_11 = 0;
    ai_12 = 1;
    ai_13 = 0;
    ai_14 = 0;
    
    ai_21 = 0;
    ai_22 = -(C_f + C_r)/(m_v*i);
    ai_23 = (C_f + C_r)/m_v;
    ai_24 = (C_r*l_r - C_f*l_f)/(m_v*i);
    
    ai_31 = 0;
    ai_32 = 0;
    ai_33 = 0;
    ai_34 = 1;
    
    ai_41 = 0;
    ai_42 = (C_r*l_r - C_f*l_f)/(J_psi*i);
    ai_43 = (C_f*l_f - C_r*l_r)/J_psi;
    ai_44 = -(C_r*l_r^2 + C_f*l_f^2)/(J_psi*i);
    
    b1i_1 = 0;
    b1i_2 = C_f/m_v;
    b1i_3 = 0;
    b1i_4 = (C_f*l_f)/J_psi;
    
    Ai = [ai_11 ai_12 ai_13 ai_14;
         ai_21 ai_22 ai_23 ai_24;
         ai_31 ai_32 ai_33 ai_34;
         ai_41 ai_42 ai_43 ai_44];
    
    Bi_1 = [b1i_1; b1i_2; b1i_3; b1i_4];
    
    C = eye(4);
    % Modify the A, B, Q, R matrices as needed for each x if they depend on x
    [K,S,P] = lqr(Ai,Bi_1,Q,R);
    k1(i) = K(1);
    k2(i) = K(2);
    k3(i) = K(3);
    k4(i) = K(4);
   
end
x = x*3.6;

% Plotting the values
figure;
plot(x, k1, 'DisplayName', 'k1');
hold on;
plot(x, k2, 'DisplayName', 'k2');
hold on
plot(x, k3, 'DisplayName', 'k3');
hold on;
plot(x, k4, 'DisplayName', 'k4');
hold off;

% Add labels and legend
xlabel('v [km/h]');
ylabel('Gain value');
title('LQR K-gain as function of velocity');
legend show;
grid on;
