%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function state = init_states( model, state )
% by Ana de Sousa (anacsousa@lara.unb.br)
% January 2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function state = init_states_75( osimModel, state )

    % locking corresponding states and definint initial pose
    editableCoordSet = osimModel.updCoordinateSet();
    
    editableCoordSet.get('pelvis_tilt').setValue(state, 45);
    editableCoordSet.get('pelvis_tilt').setLocked(state, true);    
    editableCoordSet.get('pelvis_tx').setValue(state, -0.700);
    editableCoordSet.get('pelvis_tx').setLocked(state, true);
    editableCoordSet.get('pelvis_tx').setValue(state, 0.050);
    editableCoordSet.get('pelvis_tx').setLocked(state, true);
    editableCoordSet.get('ankle_angle_r').setValue(state, deg2rad(-0.000));
    editableCoordSet.get('ankle_angle_r').setLocked(state, true);
    editableCoordSet.get('ankle_angle_l').setValue(state, deg2rad(-0.000));
    editableCoordSet.get('ankle_angle_l').setLocked(state, true);
    editableCoordSet.get('lumbar_extension').setValue(state, deg2rad(0.000));
    editableCoordSet.get('lumbar_extension').setLocked(state, true);
    
    %r
    editableCoordSet.get('hip_flexion_r').setValue(state, deg2rad(72.437));
    editableCoordSet.get('hip_flexion_r').setLocked(state, false);
    
    editableCoordSet.get('knee_angle_r').setValue(state, deg2rad(-44.974));
    editableCoordSet.get('knee_angle_r').setLocked(state, false);
    
    %l
    editableCoordSet.get('hip_flexion_l').setValue(state, deg2rad(106.104));
    editableCoordSet.get('hip_flexion_l').setLocked(state, false);
    
    editableCoordSet.get('knee_angle_l').setValue(state, deg2rad(-111.891));
    editableCoordSet.get('knee_angle_l').setLocked(state, false);
    
    %0
    editableCoordSet.get('gearToGround_coord_0').setValue(state, deg2rad(-75.180));
    editableCoordSet.get('gearToGround_coord_0').setLocked(state, false);
    
    editableCoordSet.get('rightPedalToGear_coord_0').setValue(state, deg2rad(-32.027));
    editableCoordSet.get('rightPedalToGear_coord_0').setLocked(state, false);
    
    editableCoordSet.get('leftPedalToGear_coord_0').setValue(state, deg2rad(-77.623));
    editableCoordSet.get('leftPedalToGear_coord_0').setLocked(state, false);
    
    % recalculate the derivatives after the coordinate changes
    osimModel.computeStateVariableDerivatives(state);

end

