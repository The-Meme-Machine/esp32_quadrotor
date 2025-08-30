%% Physical

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

% RPM
% Ideal (no prop)
RPM = @(throttle) v_bat * motor_kv * throttle / 100 * 0.5; % 0.5 is the correcting factor for drag

% Coefficient of lift (prop)
% relates propeller lift to RPM


% Coefficient of drag (prop)
% relates propeller drag to RPM


% Moment of inertia tensor

% Lift force
% Equation from Electric Airplane Guy
F_l = @(throttle, v_forward) 1.225 * pi * (0.0254 * prop_diam)^2 / 4 * ((RPM(throttle) * 0.0254 * prop_pitch * 1/60) - (RPM(throttle) * 0.0254 * prop_pitch * 1/60) * v_forward) * (prop_diam / (3.29546 * prop_pitch))^1.5;

% Drag torque
Q = 0;

% Actuator model is not possible as the ESC firmware doesn't support telemetry
% At best, prop inertia can be modeled