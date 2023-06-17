clear all;close all;clc;
%%
% ----------------------------------------------- %
%  Load plant model (linearized full SS model)
% ----------------------------------------------- %

understeer_vehicle = [
    1.3; % Lr  = from G to Cf  
    1.3; % Lf = from Cr to G
    24; % KLr = rear tire cornering stiffness 
    20; % KLf = front tire cornering stiffness
    5000; % Nr  = rear load
    5000; % Nf  = front
    1000; % M   = mass
    1200]; % IG  = yaw inertia


% Create the state space here
[A,B,C,D] = state_space(understeer_vehicle,10);

%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %
%  DESIGN 2
%
%  Assumption: controlled var = psi_dot -> yaw rate
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %
sys = ss(A,B,C,D); % use the fucntion ss

% Note that sys does not have any pole at the origin

% ----------------------------------------------- %
%  Control design with Bode's method
% ----------------------------------------------- %

% Design specifications:
% 
% 1. stabilize the closed loop system with the nominal plant
%
% 2. guarantee a certain amount of robustess against modeling errors
%    (typical modeling errors = linearization errors)
%
% 3. guarantee good disturbance rejection 
%    (we will consider an output disturbance representing the effects
%     of lateral gusts on the controlled variable)

% To meet req.2 it is necessary to have good stability margins. 
% In the design, we impose a phase margin PM = 60 deg; the gain margin
% is verified at the end of the control design.
PM = 60;

% To meet req.3 we suppose that the spectrum of the lateral gust 
% has a limited bandwidth, e.g. <= 10 rad/s. We select the gain 
% crossover frequency wgc > 10 rad/s, so that the sensitivity function
% of the control loop has a mag < 0 dB in the frequency region 
% where the energy of the disturbance is max.
wgc = 30;

% USE a PI controller
% useful functions
% zpk or tf, bode

% WRITE HERE

s = tf([1],[1 0]);
sysL0 = sys*s;
[mag,phase] = bode(sysL0,wgc);

DK = 1/mag;
DP = PM - 180 - phase;

K_D = DK*sind(DP)/wgc;
K_P = DK*cosd(DP);

PI = tf([K_D K_P],1);

% check design
figure;
PIsys = PI*sysL0;
margin(PIsys);
%printpdf(gcf,strcat('margin_check'));

% Comnpute the 
sysT = feedback(PIsys,1); % complementary sensitivity tf
sysS = feedback(1,PIsys); % sensitivity tf

% check sysT, sysS
figure;
bode(sysT);
hold on;
bode(sysS,'r');
legend('T(s)','S(s)')

%%
% ----------------------------------------------- %
%  Test performance on the nonlinear model 
% ----------------------------------------------- %
% Simulate the effect of a constant lateral gust.
% Change simNL.mdl to perform other tests.
options = simset('solver', 'ode4', 'FixedStep', 1e-3);
tFin = 4; % sim final time
simout = sim(['sim_yawrate.slx'], tFin, options);

figure;
subplot(2,1,1);
plot(simout.psi_dot.time, simout.psi_dot.signals.values);
ylabel('psi\_dot');
subplot(2,1,2);
plot(simout.delta.time, simout.delta.signals.values);
ylabel('delta');
%printpdf(gcf,strcat('test_performance'));

