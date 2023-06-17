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
auxdata.g   = 9.81;  % [m/s^2]  Gravitational aceeleration
auxdata.r_w = 0.3;   % [m]      Wheel radius
auxdata.m   = 1600/4;   % [kg]     Single corner mass
auxdata.J   = 1;     % [kg m^2] Wheel inertia
auxdata.Izz = 1300;
auxdata.W   = 1.85;
auxdata.L   = 4.5;

auxdata.Lf  = 4.5/2+0.5;
auxdata.Kus = 0.01;    % [-] understeering gradient vehicle

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
auxdata.road_condition = 3;

fprintf( 'Road condition: %s.\n', ...
         road_condition_names{ auxdata.road_condition } );

%-------------------------------------------------------------------------------
%% Simulate and resample data
%-------------------------------------------------------------------------------

% Simulation parameters
t0         = 0;           % [s] Initial simulation time
tf         = 20;           % [s] Final simulation time
dt         = 1e-3;        % [s] Time interval
time       = (t0:dt:tf)'; % [s] Time vector

% Run simulation
sim( 'ESP_model' );

% Extract simulation time
simTime = ESP.omega.Time;

% Vehicle with ESP (interpolation)
omega_ESP      = interp1( simTime, ESP.omega.Data,      time );
omega_ref_ESP  = interp1( simTime, ESP.omega_ref.Data,      time );
lambda_ESP     = interp1( simTime, ESP.lambda.Data, time );
Fx_ESP         = interp1( simTime, ESP.Fx.Data,     time );
Fy_ESP         = interp1( simTime, ESP.Fy.Data,     time );

% Vehicle without ESP (interpolation
omega_noESP      = interp1( simTime, noESP.omega.Data,      time );
omega_ref_noESP  = interp1( simTime, noESP.omega_ref.Data,      time );
lambda_noESP     = interp1( simTime, noESP.lambda.Data, time );
Fx_noESP         = interp1( simTime, noESP.Fx.Data,     time );
Fy_noESP         = interp1( simTime, noESP.Fy.Data,     time );

%-------------------------------------------------------------------------------
%% Plot the omega and omega_ref
%-------------------------------------------------------------------------------

% Initialize figure
figure( 'Name', 'Omega and Omega ref', 'NumberTitle', 'off' );

hold on;
grid on;

plot( time, omega_ESP, '-.r', 'LineWidth', 2 );
plot( time, omega_noESP, '-.b', 'LineWidth', 2 );
plot( time, omega_ref_ESP, '--k', 'LineWidth', 2 );

ylabel( '$\Omega [rad/s]$' );
xlabel( '$Time [s]$'    );
legend( {'$\omega_{ESP}$', '$\omega_{noESP}$', '$\omega_{refESP}$'} )

%-------------------------------------------------------------------------------
%% Plot the Forces
%-------------------------------------------------------------------------------

% Initialize figure
figure( 'Name', 'Fx and Fy', 'NumberTitle', 'off' );

hold on;
grid on;

plot( time, Fx_ESP, '-.r', 'LineWidth', 2 );
plot( time, Fy_ESP, ':r', 'LineWidth', 2 );
plot( time, Fx_noESP, '--b', 'LineWidth', 2 );
plot( time, Fy_noESP, ':b', 'LineWidth', 2 );

ylabel( '$F [N]$' );
xlabel( '$Time [s]$'    );
legend( {'$Fx_{ESP}$', '$Fy_{ESP}$', '$Fx_{noESP}$', '$Fy_{noESP}$'} )
