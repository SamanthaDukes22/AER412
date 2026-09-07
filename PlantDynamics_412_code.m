%% 1. Define variables for time, angular velocities, and torques 
syms t
syms w_x(t) w_y(t) w_z(t)             % body ang vel
syms omega_x(t) omega_y(t) omega_z(t) % rw ang vel
syms tau_ext_x tau_ext_y tau_ext_z     % external torque disturbance

%% 2. Define the Inertia Matrices
syms I_xx I_yy I_zz % body inertia
I = [I_xx, 0,    0; 
     0,    I_yy, 0; 
     0,    0,    I_zz];

syms i_xx i_yy i_zz % rw inertia
I_rw = [i_xx, 0,    0; 
        0,    i_yy, 0; 
        0,    0,    i_zz];

% Numerical Reaction Wheel-to-Body rotation matrix
R_rw_b = [-sqrt(2)/2,  1/2,        1/2; 
           0,         -sqrt(2)/2,  sqrt(2)/2; 
           sqrt(2)/2,  1/2,        1/2];

R_b_rw = transpose(R_rw_b); 

%% 3. Construct vectors 
w = [w_x(t); w_y(t); w_z(t)];
w_dot = diff(w, t); 

omega = [omega_x(t); omega_y(t); omega_z(t)]; % Fixed to column vector
omega_dot = diff(omega, t);


%% 4. Dynamic equations 

% Reaction wheel + motor dynamic equation
% First-order model

syms omega_x_cmd omega_y_cmd omega_z_cmd

syms T_x T_y T_z

omega_dot_act = [
    (omega_x_cmd - omega_x(t))/T_x;
    (omega_y_cmd - omega_y(t))/T_y;
    (omega_z_cmd - omega_z(t))/T_z
    ];

omega_dot = omega_dot_act;

%{
omega_dot_act = [
    (omega_x(t) - omega_x(t-1))/ t_s;
    (omega_y(t) - omega_y(t-1))/ t_s;
    (omega_z(t) - omega_z(t-1))/ t_s
    ];

omega_dot = omega_dot_act;
%}

% Second order dynamic model
tau_ext = I * w_dot + cross(w, I * w) + ...
         (R_b_rw * (I_rw * omega_dot)) + cross(w, R_b_rw * (I_rw * omega));


%% 5. Separate the vector equation into 3 directions
Eq_X = tau_ext_x == tau_ext(1);
Eq_Y = tau_ext_y == tau_ext(2);
Eq_Z = tau_ext_z == tau_ext(3);


%% 6. Display Dynamic Equations
disp('3 Equations of motion:')
disp('X-Direction Equation')
disp(Eq_X)

disp('Y-Direction Equation')
disp(Eq_Y)

disp('Z-Direction Equation')
disp(Eq_Z)

%% 7. Display reaction wheel equations

disp('Reaction Wheel + Motor Equations:')

disp('X Wheel:')

disp(omega_dot_act(1))

disp('Y Wheel:')

disp(omega_dot_act(2))

disp('Z Wheel:')

disp(omega_dot_act(3))

%% 8. State-Space Model

% State vector
x = [
    w_x(t);
    w_y(t);
    w_z(t);
    omega_x(t);
    omega_y(t);
    omega_z(t)
];

% Input vector - commanded reaction wheel ang vel
u = [
    omega_x_cmd;
    omega_y_cmd;
    omega_z_cmd
];


% External torques are set to zero 
tau_ext = [0; 0; 0];

% Body dynamics
w_dot_ss = simplify(I \ ... % I^-1(x)
                   (tau_ext ...
                    - cross(w, I * w) ...
                    - R_rw_b* (I_rw * omega_dot) ...
                    - cross(w, R_rw_b * (I_rw * omega)) ));

% State derivative vector x_dot = f(x,u)
f = [
    w_dot_ss;
    omega_dot_act
];          


%% 9. Linearized Conditions

% State operating point
% [wx wy wz omega_x omega_y omega_z]
x0 = [
    0;
    0;
    0;
    0;
    0;
    0
];

% Input operating point
% [omega_x_cmd omega_y_cmd omega_z_cmd]
u0 = [
    0;
    0;
    0
];


%% 10. Replace time-dependent variables with state variables

syms X1 X2 X3 X4 X5 X6
syms U1 U2 U3

X = [
    X1;
    X2;
    X3;
    X4;
    X5;
    X6
];

U = [
    U1;
    U2;
    U3
];

f_state = subs(f, ...
    [w_x(t), w_y(t), w_z(t), ...
     omega_x(t), omega_y(t), omega_z(t), ...
     omega_x_cmd, omega_y_cmd, omega_z_cmd], ...
    [X1, X2, X3, ...
     X4, X5, X6, ...
     U1, U2, U3]);


%% 11. Calculate State-Space Matrices

A = jacobian(f_state, X); % A = partial f wrt x

B = jacobian(f_state, U); % B = partial f wrt u


%% 12. Substitute Operating Point: Symbolic Matrices

A_op = simplify(subs(A, ...
    [X1 X2 X3 X4 X5 X6 U1 U2 U3], ...
    [x0' u0']));

B_op = simplify(subs(B, ...
    [X1 X2 X3 X4 X5 X6 U1 U2 U3], ...
    [x0' u0']));


%% 13. Substitute Numerical Parameters: Numeric Matrices

A_num = double(subs(A_op, ...
    [I_xx I_yy I_zz ...
     i_xx i_yy i_zz ...
     T_x T_y T_z], ...
    [1 1.2 7 ...
     0.1 0.2 0.8 ...
     0.1 0.1 0.1]));

B_num = double(subs(B_op, ...
    [I_xx I_yy I_zz ...
     i_xx i_yy i_zz ...
     T_x T_y T_z], ...
    [1 1.2 7 ...
     0.1 0.2 0.8 ...
     0.1 0.1 0.1]));


%% 14. Display State-Space Matrices

disp('======================================')
disp('OPERATING POINT')
disp('======================================')

disp('x0 =')
disp(x0)

disp('u0 =')
disp(u0)


disp('======================================')
disp('A MATRIX - SYMBOLIC')
disp('======================================')

disp(A_op)


disp('======================================')
disp('B MATRIX - SYMBOLIC')
disp('======================================')

disp(B_op)


disp('======================================')
disp('A MATRIX - NUMERICAL')
disp('======================================')

disp(A_num)


disp('======================================')
disp('B MATRIX - NUMERICAL')
disp('======================================')

disp(B_num)


%% 15. Output Equation

% Output all 6 states
% Will eventually be changed to only body

C = eye(6); 

% No direct feedthrough from input to output
% i.e. Commanded ang vel of rw is not instantaneous

D = zeros(6,3);


disp('======================================')
disp('C MATRIX')
disp('======================================')

disp(C)


disp('======================================')
disp('D MATRIX')
disp('======================================')

disp(D)

%% 16. Transfer Function

% G(s) = C( ( (sI - A)^-1) B + D) )

% Calculate the transfer function for the body dynamics, this is the one we
% will use bc we don't care abt ang vel of rw for the plant dynamics where

% Inputs: [omega_x_cmd omega_y_cmd omega_z_cmd]
% Outputs: [wx wy wz]

C_body = [eye(3), zeros(3,3)];
D_body = zeros(3,3);

sys_body = ss(A_num, B_num, C_body, D_body);

G_body = tf(sys_body);


for i = 1:3
    for j = 1:3
        fprintf('\nG(%d,%d) = \n', i, j);
        G_body(i,j)
    end
end

% Mapping input to output:

    % G(1,1) = w_x / omega_cmd_x
    % G(2,1) = w_y / omega_cmd_x
    % G(3,1) = w_z / omega_cmd_x
    
    % G(1,2) = w_x / omega_cmd_y
    % G(1,3) = w_x / omega_cmd_z

%% 17. Eigenvalues

%lamda = eig(A)

eigenvalues = eig(A_num);

disp('Eigenvalues:')
disp(eigenvalues)

%% 18. Control System Variables

euler_cmd = [0;  %roll
             0;  %pitch
             0]; %yaw

%% Fake Reaction Wheel Model

Ts = 0.01;      % Sample time [s]
tau = 0.1;      % Motor time constant [s]

% Continuous first-order model
G_rw_cont = tf(1,[tau 1]);

% Convert to discrete
G_rw = c2d(G_rw_cont, Ts, 'zoh');

% 3 identical reaction wheels
G_RW_matrix = blkdiag(G_rw, G_rw, G_rw);

disp('Reaction Wheel Transfer Function')
G_RW_matrix;

