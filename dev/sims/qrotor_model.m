function [Xdot] = qrotor_model(X, U)
%% State
% x = [
% x
% y
% z
% u
% v
% w
% phi
% theta
% psi
% p
% q
% r
% ];

%% Input
% 2000 steps of throttle in DSHOT
% u = [
% th1
% th2
% th3
% th4
% ];

%% Constants

g = 9.81;

% Motor size
motor_size = 1404;

% Motor KV
motor_kv = 3500;

% Mass
m = 0.210; % kgrams

% Propeller diam
prop_diam = 3.5;

% Propeller pitch
prop_pitch = 2.5;

% Number of blades
prop_blades = 3;

% Battery size
batt = 4;%S0P

% Battery voltage
v_bat = 3.8 * batt;

% Moment arm length (x dir)
l_x = 0.050; % meters

% Moment arm length (y dir)
l_y = 0.050;

% Moment arm length (z dir)
l_z = 0;

% IMU distance from CoG
r_IMU = [0.012,0,0]; % x,y,z (in mm)

% k_f relates propeller lift to propeller thrust level
% k_f = 12;

% k_d relates propeller aerodynamic drag to propeller thrust level
k_d = 0.5;

% Coefficient of drag of total system
C_d = 0.1;

% TODO: experimentally validate inertia tensor
% These are approximate test values
J_b = [
    0.000697 0 0
    0 0.000697 0
    0 0 0.000553
    ];

invJ_b = inv(J_b);

%% Split up state vector
% Velocity in the body frame
v_b = [
    X(4)
    X(5)
    X(6)
    ];

% Rotation of the body frame wrt the inertial frame
wbi_b = [
    X(10)
    X(11)
    X(12)
    ];

phi = X(7);
theta = X(8);
psi = X(9);


%% Rotation from inertial frame to body frame
Cb2 = [
    cos(psi) sin(psi) 0
    -sin(psi) cos(psi) 0
    0 0 1
    ];

C21 = [
    cos(theta) 0 -sin(theta)
    0 1 0
    sin(theta) 0 cos(theta)
    ];

C1i = [
    1 0 0
    0 cos(phi) sin(phi)
    0 -sin(phi) cos(phi)
    ];

Cbi = Cb2 * C21 * C1i;

%% Sum of Forces (N)

% Force of props + motors
% Equation from Electric Airplane Guy
RPM = v_bat * motor_kv * U / 2000 * 0.5; % 0.5 is the correcting factor for drag
v_forward = v_b(3); % forward airspeed is incoming from the z direction
F_lift = 1.225 * pi * (0.0254 * prop_diam)^2 / 4 * ((RPM * 0.0254 * prop_pitch * 1/60).^2 - (RPM * 0.0254 * prop_pitch * 1/60) * v_forward) * (prop_diam / (3.29546 * prop_pitch))^1.5;
Fmotor_b = [
    0
    0
    sum(F_lift)
    ];

% Force of gravity
Fgrav_i = [
    0
    0
    -g * m
    ];

% Drag force (does not consider geometry)
Fdrag_b = v_b.^2 * -C_d;

% Transform force of gravity to body frame
Fgrav_b = Cbi * Fgrav_i;

% Sum of forces in body frame
F_b = Fgrav_b + Fmotor_b + Fdrag_b;

%% Sum of Moments (Nm)
M_b = [
    [-l_x -l_x l_x l_x] * U % x dir
    [l_y -l_y -l_y l_y] * U % y dir
    sum([k_d -k_d k_d -k_d] * U) % z dir
    ];

% TODO: Transient moments as props spin up

%% Acceleration

% Coriolis accel
vb_dot = F_b / m - cross(wbi_b, v_b);

%% Angular Acceleration

wbi_b_dot = invJ_b * (M_b - cross(wbi_b, J_b * wbi_b));

%% Euler Angles
% TODO: Convert state to quats
Q = eul2quat([phi, theta, psi], "XYZ");

Omega = [
    0 -wbi_b(1) -wbi_b(2) -wbi_b(3)
    wbi_b(1) 0 wbi_b(3) -wbi_b(2)
    wbi_b(2) -wbi_b(3) 0 wbi_b(1)
    wbi_b(3) wbi_b(2) -wbi_b(1) 0
    ];

Q_dot = 0.5 * Omega * Q';

% Check if zero rotation due to quat2eul function
if norm(Q_dot) == 0
    Q_dot = [1; 0; 0; 0];
end

%% Assemble X_dot

Xdot(1:3) = X(4:6);

Xdot(4:6) = vb_dot;

Xdot(7:9) = quat2eul(Q_dot', "XYZ");

Xdot(10:12) = wbi_b_dot;

Xdot = transpose(Xdot);

end