clear;

global Tf
global flagPosition
global flagMusclesQUAD
global flagMusclesHAMS
global flagMusclesGLUT
global flagKneeTorque
global flagLoad
global flagKneeStart
global flagKneeRange
global flagFAT_R
global flagFAT_L
global flagMotor
global controlType
global refVel1

%% SIMULATION PARAMETERS
% general constants
Tf = 5;                % duration (in seconds)
flagPosition = 0;       % initial position of the right foot
flagMotor = 0;          % 0 - no motor aid / =!0 - motor aid [0,1]
controlType = 'PID';    % define controller
flagMusclesQUAD = 1;    % turn on/off quadriceps
flagMusclesHAMS = 0;    % turn on/off hamstrings
flagMusclesGLUT = 0;    % turn on/off glutous 
flagLoad = 0;           % 0 - no load at the crankset / =!0 load at the crankset
flagFAT_R = 0;          % 0 - no fatigue / =!0 - fatigue constant (higher this value, lower the fatigue)
flagFAT_L = 0;          % 0 - no fatigue / =!0 - fatigue constant (higher this value, lower the fatigue)

% spring constants
flagKneeTorque = 0.35;  % max torque (c.f., Sousa2019) / 0 - no passive orthoses
flagKneeStart = 92;     % start angle (c.f., Sousa2019)
flagKneeRange = 112-flagKneeStart;  % rangle angles (c.f., Sousa2019)

% control parameters
refVel1 = 360;          % reference speed for the controller

%% RUN
cycling_script;

