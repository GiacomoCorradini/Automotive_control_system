function plant_sfun(block)

%
% The setup method is used to setup the basic attributes of the
% S-function such as ports, parameters, etc. Do not add any other
% calls to the main body of the function.
%
setup(block);

%endfunction


function setup(block)
%% Function: setup ===================================================
% Abstract:
%   Set up the S-function block's basic characteristics such as:
%   - Input ports
%   - Output ports
%   - Dialog parameters
%   - Options
%

% Register number of ports
block.NumInputPorts  = 1;
block.NumOutputPorts = 1;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Override input port properties
block.InputPort(1).Dimensions  = 1;
block.InputPort(1).DatatypeID  = 0;  % double
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).DirectFeedthrough = true;

% Override output port properties
block.OutputPort(1).Dimensions  = 1;
block.OutputPort(1).DatatypeID  = 0; % double
block.OutputPort(1).Complexity  = 'Real';

block.OutputPort(1).SamplingMode = 'Sample';
  % SetInputPortSamplingMode:
  %   Functionality    : Check and set input and output port 
  %                      attributes and specify whether the port is operating 
  %                      in sample-based or frame-based mode
  %   C-Mex counterpart: mdlSetInputPortFrameData.
  %   (The DSP System Toolbox is required to set a port as frame-based)
  %
  block.RegBlockMethod('SetInputPortSamplingMode', @SetInpPortFrameData);

% Register parameters
block.NumDialogPrms     = 0;

% Register sample times
%  [0 offset]            : Continuous sample time
%  [positive_num offset] : Discrete sample time
%
%  [-1, 0]               : Inherited sample time
%  [-2, 0]               : Variable sample time
block.SampleTimes = [0 0];

% Setup Dwork
block.NumContStates = 3;

% -----------------------------------------------------------------
% The M-file S-function uses an internal registry for all
% block methods. You should register all relevant methods
% (optional and required) as illustrated below. You may choose
% any suitable name for the methods and implement these methods
% as local functions within the same file. See comments
% provided for each function for more information.
% -----------------------------------------------------------------

block.RegBlockMethod('InitializeConditions', @InitializeConditions);
block.RegBlockMethod('Outputs', @Outputs);     % Required
block.RegBlockMethod('Derivatives', @Derivatives);
block.RegBlockMethod('Terminate', @Terminate); % Required
%end setup


function InitializeConditions(block)

block.ContStates.Data = [10; 0; 0];
%end InitializeConditions


function Outputs(block)

x = block.ContStates.Data;
block.OutputPort(1).Data = x(3);
%end Outputs


function Derivatives(block)
pp = [
    1.3; % Lr  = from G to Cf
    1.3; % Lf = from Cr to G
    24; % KLr = rear tire cornering stiffness
    24; % KLf = front tire cornering stiffness
    5000; % Nr  = rear load
    5000; % Nf  = front
    1000; % M   = mass
    1200]; % IG  = yaw inertia

Lr = pp(1);
Lf = pp(2);
KLr = pp(3);
KLf = pp(4);
Nr = pp(5);
Nf = pp(6);
M = pp(7);
IG = pp(8);

Dyr = 1.2; % rear tire Pacejeka D coefficient
Dyf = 1.2; % front

Sr = 0;
FD = 0;

x = block.ContStates.Data;
delta = block.InputPort(1).Data;

Fr = KLr * atan((x(3)*Lr-x(2))/x(1)) * Nr / ...
    sqrt((Dyr^2+KLr^2*atan((x(3)*Lr-x(2))/x(1))^2)/Dyr^2);


Ff = -KLf*...
    sqrt(...
    (Dyf^2+ KLf^2*...
    atan( (x(3)*Lf*cos(delta)+cos(delta)*x(2)-sin(delta)*x(1))/...
    (x(3)*Lf*sin(delta)+sin(delta)*x(2)+cos(delta)*x(1))...
    )^2 ...
    )/Dyf^2 ...
    )*Nf* ...
    atan( ...
    (x(3)*Lf*cos(delta)+cos(delta)*x(2)-sin(delta)*x(1)) / ...
    (x(3)*Lf*sin(delta)+sin(delta)*x(2)+cos(delta)*x(1)) ...
    );

dotx = zeros(3,1);
dotx(1) = (M * x(3) * x(2) + Sr - Ff * sin(delta) - FD) / M;
dotx(2) = -(M * x(3) * x(1) - Fr - Ff * cos(delta)) / M;
dotx(3) = -(Lr * Fr - Lf * Ff * cos(delta)) / IG;

block.Derivatives.Data = dotx;
%end Derivatives

function SetInpPortFrameData(block, idx, fd)
  
  block.InputPort(idx).SamplingMode = fd;
  block.OutputPort(1).SamplingMode  = fd;

function Terminate(block)
%end Terminate

