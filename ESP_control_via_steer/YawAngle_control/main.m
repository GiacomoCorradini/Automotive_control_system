clear all; close all; clc;
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
    5000; % Nf  = front load
    1000; % M   = mass
    1200]; % IG  = yaw inertia

% Create the state space here
% [A,B,C,D] =

% QUESTION: What is the controlled var: psi_dot or psi ??

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %
%  DESIGN 1
%
%  Assumption: controlled var = psi
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %
sys = 1; % use ss function or/and zpk

%%

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

% Design your PD

% CODE HERE
PD = tf(1);

%%
% ----------------------------------------------- %
%  Design check on the nominal model
% ----------------------------------------------- %

sysL = PD*sysP; % loop tf
sysT = feedback(sysL,1); % complementary sensitivity tf
sysS = feedback(1,sysL); % sensitivity tf
%
%                                d(t)
%                                |
% r(t)-->[+]-->[C(s)]-->[P(s)]--[+]--o-->y(t)
%         |                          |
%       - |                          |
%         |                          |
%         +--------------------------+
%
%  Sensitivity tf:  S(s) = Y(s)/D(s)
%  Complementary sensitivity tf:  T(s) = Y(s)/R(s)

% check margins
figure;
margin(sysL);
%printpdf(gcf,strcat('margin_check'));


% check sysT, sysS
figure;
bode(sysT);
hold on;
bode(sysS,'r');
legend('T(s)','S(s)')
%printpdf(gcf,strcat('sensitivity'));


%%
% ----------------------------------------------- %
%  Test performance on the nonlinear model 
% ----------------------------------------------- %

% Simulate the effect of a constant lateral gust.

% Change simNL.mdl to perform other tests.
options = simset('solver', 'ode4','FixedStep', 1e-3);
tFin = 4; % sim final time
sim('sim_yawangle.slx', tFin, options);

figure;
subplot(3,1,1);
plot(psi.time, psi.signals.values);
ylabel('psi');
subplot(3,1,2);
plot(psi_dot.time, psi_dot.signals.values);
ylabel('psi\_dot');
subplot(3,1,3);
plot(delta.time, delta.signals.values);
ylabel('delta');
% printpdf(gcf,strcat('test_performance'));




