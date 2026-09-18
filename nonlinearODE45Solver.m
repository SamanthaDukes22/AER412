%% ODE45 to solve 9-state equation
Ixx = 1;
Iyy = 1;
Izz = 1;

Jxx = 1; % Izz of rw 1
Jyy = 1; % Izz of rw 2
Jzz = 1; % Izz of rw 3

% Constant external torque acting on the system
tau_ext_num = [0;0;0];

% Commanded reaction wheel angular acceleration
Omega_dot_cmd = [1;0;0];

% x =
%
% [ roll (phi)
%   pitch (theta)
%   yaw (psi)
%   wx
%   wy
%   wz
%   Omega_x
%   Omega_y
%   Omega_z ]

x0 = zeros(9,1);

tspan = [0 10];

[t,x] = ode45( ...
    @(t,x) nonlinearPlant( ...
        t,x,...
        Ixx,Iyy,Izz,...
        Jxx,Jyy,Jzz,...
        tau_ext_num,...
        Omega_dot_cmd), ...
    tspan,...
    x0);


wx = x(:,1);
wy = x(:,2);
wz = x(:,3);

Omega_x = x(:,4);
Omega_y = x(:,5);
Omega_z = x(:,6);

phi   = x(:,7);
theta = x(:,8);
psi   = x(:,9);

%% Angular velocity of body plot
figure

plot(t,wx)
hold on
plot(t,wy)
plot(t,wz)

grid on

xlabel('Time (s)')
ylabel('Body Angular Velocity (rad/s)')

legend('\omega_x','\omega_y','\omega_z')

%% Angular velocity of reaction wheels

figure

plot(t,Omega_x)
hold on
plot(t,Omega_y)
plot(t,Omega_z)

grid on

xlabel('Time (s)')
ylabel('Reaction Wheel Angular Velocity (rad/s)')

legend('\Omega_x','\Omega_y','\Omega_z')

%% Roll, pitch, yaw angles

figure

plot(t,phi)
hold on
plot(t,theta)
plot(t,psi)

grid on

xlabel('Time (s)')
ylabel('Euler Angle (rad)')

legend('\phi','\theta','\psi')


%% Input variables

function dx = nonlinearPlant( ...
    ~,x,...
    Ixx,Iyy,Izz,...
    Jxx,Jyy,Jzz,...
    tau_ext,...
    Omega_dot_cmd)


I = [Ixx 0 0;
    0 Iyy 0;
    0 0 Izz];

I_rw = [Jxx 0 0;
        0 Jyy 0;
        0 0 Jzz];


R_rw_b = [-sqrt(2)/2   1/2         1/2;
            0       -sqrt(2)/2   sqrt(2)/2;
        sqrt(2)/2       1/2        1/2];

R_b_rw = transpose(R_rw_b);


%% Defined States

wx = x(1);
wy = x(2);
wz = x(3);

Omega_x = x(4);
Omega_y = x(5);
Omega_z = x(6);

phi = x(7);
theta = x(8);
psi = x(9);


w = [wx;wy;wz];

Omega = [Omega_x;Omega_y;Omega_z];

Omega_dot = Omega_dot_cmd;


w_dot = I \ ( ...
    tau_ext ...
    - cross(w,I*w) ...
    - R_b_rw*(I_rw*Omega_dot) ...
    - cross(w,R_b_rw*(I_rw*Omega)) );

phi_dot = ...
    wx ...
    + wy*sin(phi)*tan(theta) ...
    + wz*cos(phi)*tan(theta);

theta_dot = ...
    wy*cos(phi) ...
    - wz*sin(phi);

psi_dot = ...
    (wy*sin(phi) ...
    + wz*cos(phi)) / cos(theta);


dx = [
    w_dot;
    Omega_dot;
    phi_dot;
    theta_dot;
    psi_dot
];

end