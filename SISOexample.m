%% Model Predictive Control with 1 inputs and 1 output - SISO
% This code contains the MPC SISO using CPU, memory and disk
% Code made by Maurício M. Neto @maumneto@gmail.com

clc
clear all

%% Parameters Settings
% These parameters are related to the form: x(t+1) = A.x(t) + B.u(t)
% and y(t) = C.x(t) + D.u(t)

plantCPU = tf([-1397 2.379e04],[1 21.31 12.6]);                                            % CPU transfer function 
plantMEM = tf([-2.062e04 -1594 -7016 80.82],[1 6.645 2.589 2.385 0.5684]);                % Memory transfer function 
plantDISK = tf([-4.112e06 -3.035e05 -1.114e06 2.088e05],[1 3.141e04 3384 1.009e04 185.2]); % Disk transfer function 

Ts = .1;   % Time sampling
p = 1200;    % Prediction horizon
m = 2;      % Control horizon

sys = ss(plantDISK);
modelDISK = c2d(sys,Ts);

%% Create the controller object with sampling period, prediction and control horizons:
% This object has as parameters the model plant, time sampling, prediction horizon and control horizon  
% Also defines the MAX and MIN

%mpcobjCPU = mpc(modelCPU, Ts, p, m);   % MPC object for CPU input 
%mpcobjMEM = mpc(plantMEM, Ts, p, m);   % MPC object for Memory input
mpcobjDISK = mpc(modelDISK, Ts, p, m);  % MPC object for Disk input

mpcobj.MV = struct('Min',0,'Max',1);
mpcobj.MV.RateMax = 10;
mpcobj.MV.RateMin = -10;

%% Simulation parameters
Tstop = 30;                               % time simulation
Tf = round(Tstop/Ts);                     % Number of simulation steps
v=[];                                     % Distubance (zero disturbance)
%r = ones(Tf,1);                           % Reference Signal
r = repmat(300,300,1);      % 300ms step

[y,t,u] = sim(mpcobjDISK, Tf, r, v);                     % Show simulation graphs
%sim(mpcobjCPU, Tf, r, v);                     % Show simulation graphs
%sim(mpcobjMEM, Tf, r, v);                     % Show simulation graphs
sim(mpcobjDISK, Tf, r, v);                    % Show simulation graphs

% Mean Absolute Percentage Error - MAPE
errorMAPE = r-y;
MAPE = mean(abs(errorMAPE./r));  
disp('')
disp(' ---------------------------------------------------------- ')
disp('Mean Absolute Percentage Error - MAPE')
disp(' ---------------------------------------------------------- ')
disp('')
fprintf('The MAPE value = %g \n', MAPE*100);
disp(' ---------------------------------------------------------- ')
disp('')
