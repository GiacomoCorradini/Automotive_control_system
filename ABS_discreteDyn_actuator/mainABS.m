%-------------------------------------------------------------------------------
%% Initialization
%-------------------------------------------------------------------------------

clear; close all; clc;

% Define LaTeX as interpreter for titlr, labels and legend in plots
set( 0,     'defaulttextinterpreter',          'latex' );
set( groot, 'defaultAxesTickLabelInterpreter', 'latex' );
set( groot, 'defaultLegendInterpreter',        'latex' );

% Add the path to the figures and functions
addpath( 'figures' );
addpath( 'functions' );

% Define the plot speed
frame_skip = 20;

%-------------------------------------------------------------------------------
%% Parameter definition
%-------------------------------------------------------------------------------

% Longitudinal slip range
lambda = linspace( 0, 1, 100 ); % [-] lambda vector

% Vehicle parameters
tau_delay   = 10e-3; % [ms]     Delay of the actuation system (caliper)
omega_act   = 70;    % [rad/s]  Bandwidth of the actuation system (caliper)
omega_LPF   = 100;   % [rad/s]  Bandwidth of the LowPass filter
auxdata.r_w = 0.3;   % [m]      Wheel radius
auxdata.m   = 225;   % [kg]     Single corner mass
v_length    = 4.387; % [m]      Vehicle length
width       = 1.768; % [m]      Vehicle width
auxdata.g   = 9.81;  % [m/s^2]  Gravitational aceeleration
m_wheel     = 5;     % [kg]     Wheel mass
auxdata.J   = 1;     % [kg m^2] Wheel inertia
init_speed  = 100;   % [km/h]   Initial vehicle speed

% Select road condition for the simulation:
%  - road_condition = 1 -> Dry Asphalt;
%  - road_condition = 2 -> Wet Asphalt;
%  - road_condition = 3 -> Cobblestone;
%  - road_condition = 4 -> Snow.
%
road_condition_names   = { 'Dry Asphalt', ...
                           'Wet Asphalt', ...
                           'Cobblestone', ...
                           'Snow' };
auxdata.road_condition = 1;

fprintf( 'Road condition: %s.\n', ...
         road_condition_names{ auxdata.road_condition } );

% Braking torque rate
k = 5e3; % [Nm/s^2]

% Select violation of condition for the existence of the limit cycle:
%  - violate_condition = 1 -> violate no condition;
%  - violate_condition = 2 -> violate condition 1;
%  - violate_condition = 3 -> violate condition 2;
%  - violate_condition = 4 -> violate condition 3.
%
violate_condition = 1;

switch violate_condition
    case 1
        lambda_min = 0.08;
        lambda_max = 0.25;
        Tb_min     = 1000;
        Tb_max     = 1400;
    case 2
        lambda_min = 0.08;
        lambda_max = 0.25;
        Tb_min     = 1000;
        Tb_max     = 1150;
    case 3
        lambda_min = 0.08;
        lambda_max = 0.25;
        Tb_min     = 1250;
        Tb_max     = 1400;
    case 4
        lambda_min = 0.08;
        lambda_max = 0.25;
        Tb_min     = 1150;
        Tb_max     = 1400;
end

v_on       = 2.5;  % [m/s] Threshold to turn ON the ABS in high speed or low
                   %       speed mode
hv         = 0.5;  % [m/s] Threshold to switch between high speed OFF and low 

%-------------------------------------------------------------------------------
%% Plot the friction coefficients at different road conditions
%-------------------------------------------------------------------------------

figure( 'Name', 'Friction Coefficient', 'NumberTitle', 'off' );

hold on;
grid on;

xlabel( '$\lambda$'      );
ylabel( '$\mu(\lambda)$' );

title( 'Friction coefficient for different road conditions' );

for i = 1:numel( road_condition_names )
    plot( lambda, burckhardt( lambda, i ) );
end

legend( road_condition_names );

pause( 1 ); % otherwise MATLAB overwrites this figure sometimes

%-------------------------------------------------------------------------------
%% Simulate and resample data
%-------------------------------------------------------------------------------

% Simulation parameters
t0         = 0;           % [s] Initial simulation time
tf         = 4;         % [s] Final simulation time
dt         = 1e-3;        % [s] Time interval
time       = (t0:dt:tf)'; % [s] Time vector

% Select the manoeuvre type:
%  - manoeuvre_type = 1 -> simple braking test;
%  - manoeuvre_type = 2 -> piecewise braking;
%  - manoeuvre_type = 3 -> piecewise braking and hold at lower than max value.
%
manoeuvre_type = 2;
rise_time_tb   = Tb_max/k; % time needed to go from 0 to Tb_max

switch manoeuvre_type
    case 1
        % Vector of times where to switch value of the input signal
        time_manoeuvre   = [0, 1, ...
                            1 + rise_time_tb, tf];

        % Output value of the input signal at the corresponding time
        output_manoeuvre = [0, 0, ...
                            Tb_max, Tb_max];

        % Times where the vehicle starts braking
        brake_times      = [1];

        % Times where the vehicle stops braking (or releases the brake pedal)
        no_brake_times   = [];
    case 2
        % Vector of times where to switch value of the input signal
        time_manoeuvre   = [0, 1,                               ...
                            1 + rise_time_tb, 2 - rise_time_tb, ...
                            2, 2.5,                             ...
                            2.5 + rise_time_tb, tf];

        % Output value of the input signal at the corresponding time
        output_manoeuvre = [0, 0,           ...
                            Tb_max, Tb_max, ...
                            0, 0,           ...
                            Tb_max, Tb_max];

        % Times where the vehicle starts braking
        brake_times      = [1, 2.5];

        % Times where the vehicle stops braking (or releases the brake pedal)
        no_brake_times   = [2 - rise_time_tb];
    case 3
        % Vector of times where to switch value of the input signal
        time_manoeuvre   = [0, 1,                                   ...
                            1 + rise_time_tb, 2 - (Tb_max - 200)/k, ...
                            2, tf];

        % Output value of the input signal at the corresponding time
        output_manoeuvre = [0, 0,           ...
                            Tb_max, Tb_max, ...
                            Tb_max - 200, Tb_max - 200];

        % Times where the vehicle starts braking
        brake_times      = [1, 2];

        % Times where the vehicle stops braking (or releases the brake pedal)
        no_brake_times   = [2 - (Tb_max - 200)/k];
end

% Run simulation
sim( 'ABS_discDyn_model' );

% Extract simulation time
simTime = ABS.v.Time;

% Vehicle with ABS (interpolation)
v_ABS      = interp1( simTime, ABS.v.Data,      time );
x_ABS      = interp1( simTime, ABS.x.Data,      time );
lambda_ABS = interp1( simTime, ABS.lambda.Data, time );
Tb_ABS     = interp1( simTime, ABS.Tb.Data,     time );

% Vehicle without ABS (interpolation
v_noABS      = interp1( simTime, noABS.v.Data,      time );
x_noABS      = interp1( simTime, noABS.x.Data,      time );
lambda_noABS = interp1( simTime, noABS.lambda.Data, time );
Tb_noABS     = interp1( simTime, noABS.Tb.Data,     time );

%-------------------------------------------------------------------------------
%% Plot the long. slip (w/o ABS) during braking
%-------------------------------------------------------------------------------

% Equilibrium manifold
%               /      (1 - lambda) * J \
% psi_lambda = | r_w + ----------------  | * Fz * mu
%               \          r_w * m      /
psi_lambda = @( lambda, Fz ) (auxdata.r_w + ...
                              ((1 - lambda) .* auxdata.J) / ...
                              (auxdata.r_w * auxdata.m)) .* Fz .* ...
                              burckhardt( lambda, auxdata.road_condition );

% Initialize figure
figure( 'Name', 'Long. Wheel Slip during braking', 'NumberTitle', 'off' );

hold on;
grid on;

plot( lambda, psi_lambda( lambda, 1.5 * auxdata.m * auxdata.g ), '--k', ...
      'LineWidth', 2 );

xlabel( '$\lambda [-]$' );
ylabel( '$T_b [Nm]$'    );

yL = get( gca, 'YLim' ); % get the maximum y to use later

% Start plotting the lambda values
fprintf('Plotting the long. slip (w/o ABS) during braking.\n');

hABS   = plot( lambda_ABS(1), yL(2), 'color', ...
               'red',  'LineWidth', 2 );
hnoABS = plot( lambda_noABS(1), yL(2), 'color', ...
               'blue', 'LineWidth', 2 );

legend( [ hABS, hnoABS ], { 'ABS', 'noABS' } );

n = floor(tf/dt);
for i = 1:frame_skip:n
    hABS   = plot( [ lambda_ABS(i),   lambda_ABS(i)   ], yL, 'color', ...
                   'red',  'LineWidth', 2, 'HandleVisibility', 'off' );
    hnoABS = plot( [ lambda_noABS(i), lambda_noABS(i) ], yL, 'color', ...
                   'blue', 'LineWidth', 2, 'HandleVisibility', 'off' );

    progress_bar(1, n, frame_skip, i);

    pause(dt); % unless MATLAB will not show live the plot

    set( hABS,   'Visible', 'off' );
    set( hnoABS, 'Visible', 'off' );
end

plot( [ lambda_ABS(i),   lambda_ABS(i)   ], yL, 'color', ...
      'red',  'LineWidth', 2, 'HandleVisibility', 'off' );
plot( [ lambda_noABS(i), lambda_noABS(i) ], yL, 'color', ...
      'blue', 'LineWidth', 2, 'HandleVisibility', 'off' );

pause( 1 ); % otherwise MATLAB overwrites this figure sometimes

%-------------------------------------------------------------------------------
%% Plot the wheel torque (w/o ABS) during braking
%-------------------------------------------------------------------------------

% Initialize the figure
figure( 'Name', 'Long. Wheel Slip and Torque during braking', ...
        'NumberTitle', 'off' );

hold on;
grid on;

plot( lambda, psi_lambda( lambda, 1.5 * auxdata.m * auxdata.g ), '--k', ...
      'LineWidth', 2 );

ylim( [ 0, 1500 ] );

xlabel( '$\lambda [-]$' );
ylabel( '$T_b [Nm]$'    );

% Start plotting the torque values
fprintf('Plot the wheel torque (w/o ABS) during braking.\n');

hABS   = plot( lambda_ABS(1),   Tb_ABS(1),   'color', 'red',  ...
               'Linewidth', 2 );
hnoABS = plot( lambda_noABS(1), Tb_noABS(1), 'color', 'blue', ...
               'Linewidth', 2 );

legend( [ hABS, hnoABS ], { 'ABS', 'noABS' } );

n = floor(tf/dt);
for i = 2:frame_skip:n
    hABS   = plot( lambda_ABS(1:i),   Tb_ABS(1:i),   'color', 'red',  ...
                   'Linewidth', 2, 'HandleVisibility', 'off' );
    hnoABS = plot( lambda_noABS(1:i), Tb_noABS(1:i), 'color', 'blue', ...
                   'Linewidth', 2, 'HandleVisibility', 'off' );
    circ   = plot( lambda_ABS(i), Tb_ABS(i), 'X', 'LineWidth', 2, ...
                   'HandleVisibility', 'off' );

    progress_bar(2, n, frame_skip, i);

    pause(dt); % unless MATLAB will not show live the plot

    set( hABS,   'Visible', 'off' );
    set( hnoABS, 'Visible', 'off' );
    set( circ,   'Visible', 'off' );
end

plot( lambda_ABS(1:i),   Tb_ABS(1:i),   'color', 'red',  ...
      'Linewidth', 2, 'HandleVisibility', 'off' );
plot( lambda_noABS(1:i), Tb_noABS(1:i), 'color', 'blue', ...
      'Linewidth', 2, 'HandleVisibility', 'off' );

pause( 1 ); % otherwise MATLAB overwrites this figure sometimes

%-------------------------------------------------------------------------------
%% Plot the braking maneuver (vehicle position)
%-------------------------------------------------------------------------------

% Initialize the figure
figure( 'Name', 'Braking Maneuver Comparison', 'NumberTitle', 'off' );

hold on;
grid on;
axis equal;

xlim( [ 0, max( x_ABS(end), x_noABS(end) ) + v_length ] );
ylim( [ 0, 50 ] );

yL = get( gca, 'YLim' ); % get the maximum y to use later

xlabel( '$x$ [m]' );
ylabel( '$y$ [m]' );

title( [ 'Braking maneuver on ',  ...
         road_condition_names{ auxdata.road_condition } ] );

% Plot the braking starting line
for i = 1:length( brake_times )
    x_brake = x_ABS(time==brake_times(i));
    
    plot( [ x_brake, x_brake ], [ 0, 50 ], '--k' );
    text( x_brake + 1, 5, '$\leftarrow$ Braking starting point' );
end

% Plot the no braking starting line
for i = 1:length( no_brake_times )
    x_no_brake = x_ABS(time==no_brake_times(i));
    
    plot( [ x_no_brake, x_no_brake ], [ 0, 50 ], '--k' );
    text( x_no_brake + 1, 10, '$\leftarrow$ No braking starting point' );
end

% Start plotting the position values
fprintf('Plot the braking maneuver (vehicle position).\n');

h1 = plot( [ x_ABS(1),   x_ABS(1)   ], [ 15, 15 ], '--r', 'LineWidth', 2);
h2 = plot( [ x_noABS(1), x_noABS(1) ], [ 30, 30 ], '--b', 'LineWidth', 2);

legend( [ h1, h2 ], { 'with ABS', 'without ABS' } );

n = numel( time );
for i = 2:frame_skip:n
    plot( [ x_ABS(1),   x_ABS(i)   ], [ 15, 15 ], '--r', ...
          'LineWidth', 2, 'HandleVisibility', 'off' );
    plot( [ x_noABS(1), x_noABS(i) ], [ 30, 30 ], '--b', ...
          'LineWidth', 2, 'HandleVisibility', 'off' );

    hABS   = rectangle( 'Position', [ x_ABS(i) - v_length / 2, ...
                                      15 - width / 2, v_length, width ], ...
                        'Curvature', [ 0.2, 0.9 ], 'FaceColor', 'red' );
    hnoABS = rectangle( 'Position', [ x_noABS(i) - v_length / 2, ...
                                      30 - width / 2, v_length, width ], ...
                        'Curvature', [ 0.2, 0.9 ], 'FaceColor', 'blue' );

    progress_bar(2, n, frame_skip, i);

    pause( dt ); % unless MATLAB will not show live the plot

    set( hABS,  'Visible', 'off' );
    set( hnoABS,'Visible', 'off' );
end

rectangle( 'Position', [ x_ABS(i) - v_length / 2, ...
                         15 - width / 2, v_length, width ], ...
           'Curvature', [ 0.2, 0.9 ], 'FaceColor', 'red' );
rectangle( 'Position', [ x_noABS(i) - v_length / 2, ...
                         30 - width / 2, v_length, width ], ...
           'Curvature', [ 0.2, 0.9 ], 'FaceColor', 'blue' );

%% Wait, is it over already?
