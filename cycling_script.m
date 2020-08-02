%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RUNME.m
% by Ana de Sousa (anacsousa@lara.unb.br),Felipe Shimabuko
% and Antonio Padilha L. Bo
% Last update: February 2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic; close all; clc;
import org.opensim.modeling.*

%% GLOBAL VARIABLES
% timing
global Tf
global Ts
global n_samples

% general variables
global ZERO
global refVel
global measAngle
global measKneeAngle
global measVel
global musclesNames
global statesNames
global controlActionLoad
global refVel1

% general control variables
global controlType
global controlActionHat
global controlAction
global controlActionQuadRight
global controlActionQuadLeft
global controlActionHamsRight
global controlActionHamsLeft
global controlActionGlutRight
global controlActionGlutLeft
global tQUADactivatedRight
global tHAMSactivatedRight
global tGLUTactivatedRight
global tQUADactivatedLeft
global tHAMSactivatedLeft
global tGLUTactivatedLeft

global controlError
global controlTime
global controlIteration
global integralError

% knee
global angleKneeR
global angleKneeL

global controlActionTorqueRight
global controlActionTorqueLeft

% flags
global flagSensorNoise
global flagCorrection
global flagMusclesQUAD
global flagMusclesHAMS
global flagMusclesGLUT
global flagPosition
global flagKneeTorque
global flagLoad
global flagKneeStart
global flagKneeRange
global flagSTIM
global flagMotor
global flagFAT_R
global flagFAT_L

%% INITIAL CONFIGURATION
% ---> choose final time, the control sample time
Ts = 1/50; n_samples = floor(Tf/Ts);

% FLAGS 
flagSensorNoise = 0; %---> Adds a noisy signal to the inertial sensor
flagCorrection = 1;  % ---> Implements the speed correction (Cadence-based Phase Adjustment)

%% do not change:
ZERO = 1e-3;
refVel = zeros(1,n_samples);
measAngle = zeros(1,n_samples);
measKneeAngle = zeros(1,n_samples);
measVel = zeros(1,n_samples);
controlAction = ZERO * ones(1,n_samples);
controlActionHat = ZERO * ones(1,n_samples);
controlActionQuadRight = ZERO * ones(1,n_samples);
controlActionQuadLeft = ZERO * ones(1,n_samples);
controlActionHamsRight = ZERO * ones(1,n_samples);
controlActionHamsLeft = ZERO * ones(1,n_samples);
tQUADactivatedRight = 0;
tQUADactivatedLeft = 0;
tHAMSactivatedRight = 0;
tHAMSactivatedLeft = 0;
tGLUTactivatedRight = 0;
tGLUTactivatedLeft = 0;
controlActionGlutRight = ZERO * ones(1,n_samples);
controlActionGlutLeft = ZERO * ones(1,n_samples);
controlActionLoad = ZERO * ones(1,n_samples);
controlError = zeros(1,n_samples);
controlTime = zeros(1,n_samples);
controlIteration = 1;
integralError = 0;
angleKneeR = zeros(1,n_samples);
angleKneeL = zeros(1,n_samples);

controlActionTorqueRight=ZERO * ones(1,n_samples);
controlActionTorqueLeft=ZERO * ones(1,n_samples);

%% INITIALIZE MODEL
disp('> Initialize simulation')
% open and init model
cyclingModel = Model('Model/FESCyclingModel.osim');
cyclingStates = cyclingModel.initSystem();
cyclingModel.equilibrateMuscles(cyclingStates);

% control function handle
controlFunctionHandle = @cycling_control_v2;

% get muscles and states names
musclesNames = get_muscles_names(cyclingModel);
statesNames = get_states_names(cyclingModel);

% define initial pose
switch flagPosition
    case 0
        cyclingStates = init_states_0(cyclingModel, cyclingStates);
    case 15
        cyclingStates = init_states_15(cyclingModel, cyclingStates);
    case 30
        cyclingStates = init_states_30(cyclingModel, cyclingStates);    
    case 45
        cyclingStates = init_states_45(cyclingModel, cyclingStates);
    case 60
        cyclingStates = init_states_60(cyclingModel, cyclingStates);
    case 75
        cyclingStates = init_states_75(cyclingModel, cyclingStates);
    case 90
        cyclingStates = init_states_90(cyclingModel, cyclingStates);
    case 105
        cyclingStates = init_states_105(cyclingModel, cyclingStates);
    case 120
        cyclingStates = init_states_120(cyclingModel, cyclingStates);
    case 135
        cyclingStates = init_states_135(cyclingModel, cyclingStates);
    case 150
        cyclingStates = init_states_150(cyclingModel, cyclingStates);
    case 165
        cyclingStates = init_states_165(cyclingModel, cyclingStates);    
    otherwise
        cyclingStates = init_states_0(cyclingModel, cyclingStates);
end

%% CONFIG & RUN SIMULATION
disp('> Run simulation')

% integrate plant using Matlab integration
timeSpan = [0 Tf]; integratorName = 'ode15s'; 
integratorOptions = odeset('AbsTol', 1E-5','MaxStep',.1*Ts);

% run simulation using function from Dynamic Walking example
motionData = IntegrateOpenSimPlant(cyclingModel, controlFunctionHandle, timeSpan, ...
    integratorName, integratorOptions);

%% SHOW RESULTS
if controlIteration < n_samples
    n_samples = controlIteration;
else
    n_samples = length(measKneeAngle);
end

refVel = refVel(1:n_samples);
measAngle = measAngle(1:n_samples);
measKneeAngle = measKneeAngle(1:n_samples);
measVel = measVel(1:n_samples);
controlAction = controlAction(1:n_samples);
controlActionHat = controlActionHat(1:n_samples);
controlActionQuadRight = controlActionQuadRight(1:n_samples);
controlActionQuadLeft = controlActionQuadLeft(1:n_samples);
controlActionHamsRight = controlActionHamsRight(1:n_samples);
controlActionHamsLeft = controlActionHamsLeft(1:n_samples);
controlActionGlutRight = controlActionGlutRight(1:n_samples);
controlActionGlutLeft = controlActionGlutLeft(1:n_samples);
controlActionTorqueLeft = controlActionTorqueLeft(1:n_samples);
controlActionTorqueRight = controlActionTorqueRight(1:n_samples);
controlError = controlError(1:n_samples);
controlTime = controlTime(1:n_samples);
controlActionLoad = controlActionLoad(1:n_samples);
angleKneeR = angleKneeR(1:n_samples);
angleKneeL = angleKneeL(1:n_samples);

cycling_plot;

%% CREATE .STO FILE FOR VISUALIZATION, SAVE WORKSPACE AND SAVE FIGURE
Nsamples = length(motionData.data(:,1)); Nstates = length(statesNames);

% name of the file
str_name = 'cycling_';
str_name = strcat(str_name,'_Tf',num2str(Tf),'_P',num2str(flagPosition),'_Kmax',num2str(flagKneeTorque),...
    '_s',num2str(flagKneeStart),'_r',num2str(flagKneeRange),'_L',num2str(flagLoad),'_M',num2str(flagMotor),...
    '_STIM',num2str(flagSTIM),'_QHG',num2str(flagMusclesQUAD),num2str(flagMusclesHAMS),num2str(flagMusclesGLUT),...
    '_C_',controlType, '_R', num2str(refVel1), '_FatRL', num2str(flagFAT_R),'_', num2str(flagFAT_L));

    % create .STO
    str = strjoin(statesNames,'\t');
    header = ['cycling_simulation \nversion=1 \nnRows=' num2str(Nsamples) ' \nnColumns=' num2str(Nstates+1) '\ninDegrees=no \nendheader \ntime	' str '\n'];

    fid = fopen(strcat('Results/',str_name,'.sto'),'wt');
    fprintf(fid,header); fclose(fid);

    fid = fopen(strcat('Results/',str_name,'.sto'),'a+');
    for i = 1:Nsamples
        fprintf(fid,'\t%f',motionData.data(i,:)); fprintf(fid,'\n');
    end
    fclose(fid);

    % save .MAT
    varlist = {'cyclingModel','cyclingStates'}; clear(varlist{:});  % clear opensim var
    save(strcat('Results\',str_name,'.mat'));                       % save data

%% The end!
fprintf('\n'); toc; disp('> THE END')
