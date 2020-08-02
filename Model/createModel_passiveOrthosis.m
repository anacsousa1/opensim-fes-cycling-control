% Import Java Library
clear; clc;
import org.opensim.modeling.*
 
%% Open the model
osimModel = Model('FESCyclingModel.osim');
osimModel.setName('FESCyclingModel');
 
%% Utilities
zeroVec3 = ArrayDouble.createVec3(0);
 
%% Add Torque Control to crankSet
% Create Torque
gear = osimModel.updBodySet().get('Gear');
ground = osimModel.updBodySet().get('ground');
 
gear_load = TorqueActuator (gear, ground, Vec3(0,0,-1), true);
gear_load.setName('gear_load');
gear_load.setOptimalForce(100.0);

% Add the force to the model
osimModel.addForce(gear_load);
 
% Print a new model file
osimModel.print('FESCyclingModel.osim');

