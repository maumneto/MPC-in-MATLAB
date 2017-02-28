%% Model Predictive Control with 3 inputs and 1 output - MISO 
%Code made by Maurício M. Neto @maumneto@gmail.com

clear all 
clc

%% Parameters Settings
% These parameters are related to the form: x(t+1) = A.x(t) + B.u(t)
% and y(t) = C.x(t) + D.u(t)

Ts = .1;  % time sampling
Ph = 30;  % Prediction Horizon
Ch = 3;   % Control Horizon

%% Generating the transfer Function and State-space model
% Through the A,B,C and D parameters, its possible make a transfer model
% and a state-space model 

% If the inputs are represented as a transfer function...  
transferFunction = tf({[-1397 2.379e04], [-2.062e04 -1594 -7016 80.82], [-4.112e06 -3.035e05 -1.114e06 2.088e05]},{[1 21.31 12.6], [1 6.645  2.589 2.385 0.5684], [1 3.141e04 3384 1.009e04 185.2]}); 
sys = ss(transferFunction);   % Transform the model into a state-space model 

%% Converting continuous-time dynamic system to discrete time system

model = c2d(sys,Ts);

%% Set signal types in MPC plant model
% Setting the signal types in MPC plant model. In this case, the signals
% are default: all inputs are manipulated variables, all outputs are
% measured outputs

model = setmpcsignals(model,'MV',[1,2,3]);

%% Create the controller object with sampling period, prediction and control horizons:
% This object has as parameters the model plant, time sampling, prediction horizon and control horizon  
% Also defines the MAX and MIN

mpcobj = mpc(model,Ts,Ph,Ch);

% Defining constraints to all inputs
for i = 1:3
mpcobj.MV(i).Min = 0;
mpcobj.MV(i).Max = 1;
mpcobj.MV(i).RateMin = -10;
mpcobj.MV(i).RateMax = 10;
end

% Inserting the noise. If the mpcobj.Model.Noise return empty, all channels have a white gaussian noise.
%mpcobj.Model.Noise = [];

% Inserting the weights of the inputs. 
mpcobj.Weights.MV = [.3 0.1 0.1];

% Inserting the weights of the inputs rates
%mpcobj.Weights.ManipulatedVariablesRate = [.1 .1 .1];

% Inserting a disturbance in the MPC model. 
%mpcobj.Model.Disturbance = tf(sqrt(1000),[1 0]);

%% Simulation parameters

Tstop = 30;                               % time simulation
Tf = round(Tstop/Ts);                     % Number of simulation steps 
%v = ones(Tf,1);                           % measured disturbed signal
%v = [zeros(Tf/3,1);ones(2*Tf/3,1)];       % Sinal de disturbio medido

% Run the closed-loop simulation and plot results.
step1 = repmat(300,60,1);      % 300ms step
step2 = repmat(500,60,1);      % 500ms step
step3 = repmat(1000,60,1);     % 1000ms step
r = vertcat(step1,step2,step3,step2,step1);     % Signal reference

sim(mpcobj,Tf,r);       % Simulation

%% Simulations results
% all possible results that mpc function can provide

DCgain = cloffset(mpcobj);
disp('DC gain: ')
disp(DCgain)
review(mpcobj);


