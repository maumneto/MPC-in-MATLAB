%% Estimate transfer function using time or frequency domain data.    
% Code made by Maurício M. Neto @maumneto@gmail.com

clear all
clc

%% Parameters settings
load('C:\Users\MaurícioM\Desktop\rubisData.mat');   % load the data of inputs/outputs
xCPU = labpcexport.cpu;        % Inputs
xMEM = labpcexport.mem;        % Inputs
xDISK = labpcexport.disk;      % Inputs
y = labpcexport.resptime;      % Outputs
% Number of poles
npCPU = 2;                     
npMEM = 4;
npDISK = 4; 
Ts = .1;                    % Time sampling

%% Creating the data

dataCPU = iddata(y,xCPU,Ts);
dataMEM = iddata(y,xMEM,Ts);
dataDISK = iddata(y,xDISK,Ts);


%% Creating the model system based on data and number of poles of the transfer function 

sysCPU = tfest(dataCPU,npCPU);
sysMEM = tfest(dataMEM,npMEM);
sysDISK = tfest(dataDISK,npDISK);
