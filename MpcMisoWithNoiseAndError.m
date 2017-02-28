%% Model Predictive Control with 3 inputs and 1 output - MISO 
%Code made by Maurício M. Neto @maumneto@gmail.com

clear all 
clc

%% Parameters Settings
% These parameters are related to the form: x(t+1) = A.x(t) + B.u(t)
% and y(t) = C.x(t) + D.u(t)

Ts = .1;  % time sampling
Ph = 1200;  % Prediction Horizon
Ch = 10;   % Control Horizon

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
mpcobj.Weights.ManipulatedVariables = [.3 0.1 0.1];

% Inserting the weights of the inpuJts rates
mpcobj.Weights.ManipulatedVariablesRate = [.1 .1 .1];

% Inserting a disturbance in the MPC model. 
%mpcobj.Model.Disturbance = tf(sqrt(1000),[1 0]);

%% Simulation parameters

Tstop = 30;                               % time simulation
Tf = round(Tstop/Ts);                     % Number of simulation steps 
v = [];                           % measured disturbed signal
%v = [zeros(Tf/3,1);ones(2*Tf/3,1)];       % Sinal de disturbio medido

% Run the closed-loop simulation and plot results.
step1 = repmat(300,60,1);      % 300ms step
step2 = repmat(500,60,1);      % 500ms step
step3 = repmat(1000,60,1);     % 1000ms step
r = vertcat(step1,step2,step3,step2,step1);     % Signal reference

[y,t,u] = sim(mpcobj,Tf,r,v);       % Simulation
sim(mpcobj, Tf, r, v);              % show simulation graphs

%% Simulations results
% all possible results that mpc function can provide

% DC gain of the MPC
disp('')
disp(' ------------------------- ')
DCgain = cloffset(mpcobj);
fprintf('DC gain = %g \n', DCgain);
review(mpcobj);
disp(' ------------------------- ')
disp('')

% % Integral of Absolute Error - IAE
% [J1, Sens1] = sensitivity(mpcobj, 'IAE', mpcobj.Weights, Tf, r, v);
% disp('')
% disp('--------------')
% disp('Sensitivity analysis IAE')
% disp('--------------')
% disp('')
% fprintf('Output weights:     dJ/dWy  = [%g]\n',Sens1.OutputVariables);
% fprintf('Input weights:      dJ/dWu  = [%g, %g, %g]\n',Sens1.ManipulatedVariables);
% fprintf('Input-rate weights: dJ/dWdu = [%g, %g, %g]\n',Sens1.ManipulatedVariablesRate);
% disp('--------------')
% disp('')
% 
% % Integral Square Error - ISE
% [J2, Sens2] = sensitivity(mpcobj, 'ISE', mpcobj.Weights, Tf, r, v);
% disp('')
% disp('--------------')
% disp('Sensitivity analysis ISE')
% disp('--------------')
% disp('')
% fprintf('Output weights:     dJ/dWy  = [%g]\n',Sens2.OutputVariables);
% fprintf('Input weights:      dJ/dWu  = [%g, %g, %g]\n',Sens2.ManipulatedVariables);
% fprintf('Input-rate weights: dJ/dWdu = [%g, %g, %g]\n',Sens2.ManipulatedVariablesRate);
% disp('--------------')
% disp('')
% 
% % Integral of Time multiply Absolute Error - ITSE
% [J3, Sens3] = sensitivity(mpcobj, 'ITSE', mpcobj.Weights, Tf, r, v);
% disp('')
% disp('--------------')
% disp('Sensitivity analysis ITSE')
% disp('--------------')
% disp('')
% fprintf('Output weights:     dJ/dWy  = [%g]\n',Sens3.OutputVariables);
% fprintf('Input weights:      dJ/dWu  = [%g, %g, %g]\n',Sens3.ManipulatedVariables);
% fprintf('Input-rate weights: dJ/dWdu = [%g, %g, %g]\n',Sens3.ManipulatedVariablesRate);
% disp('--------------')
% disp('')
% 
% % Integral Time-weighted Absolute Error - ITAE
% [J4, Sens4] = sensitivity(mpcobj, 'ITAE', mpcobj.Weights, Tf, r, v);
% disp('')
% disp('--------------')
% disp('Sensitivity analysis ITAE')
% disp('--------------')
% disp('')
% fprintf('Output weights:     dJ/dWy  = [%g]\n',Sens4.OutputVariables);
% fprintf('Input weights:      dJ/dWu  = [%g, %g, %g]\n',Sens4.ManipulatedVariables);
% fprintf('Input-rate weights: dJ/dWdu = [%g, %g, %g]\n',Sens4.ManipulatedVariablesRate);
% disp('--------------')
% disp('')

% % Root Mean Squared Error - RMSE
RMSE = sqrt(mean((y - r).^2));  
disp('')
disp(' ------------------------- ')
disp('Root Mean Squared Error - RMSE')
disp(' ------------------------- ')
disp('')
fprintf('The RMSE value = %g \n', RMSE);
disp(' ------------------------- ')
disp('')

% % Mean Absolute Error - MAE
errorAbs = y-r;
MAE = mae(errorAbs);  
disp('')
disp(' ------------------------- ')
disp('Mean Absolute Error - MAE')
disp(' ------------------------- ')
disp('')
fprintf('The MAE value = %g \n', MAE);
disp(' ------------------------- ')
disp('')

% Mean Absolute Percentage Error - MAPE
errorMAPE = y-r;
MAPE = mean(abs(errorMAPE./r));  
disp('')
disp(' ------------------------- ')
disp('Mean Absolute Percentage Error - MAPE')
disp(' ------------------------- ')
disp('')
fprintf('The MAPE value = %g \n', MAPE*100);
disp(' ------------------------- ')
disp('')

