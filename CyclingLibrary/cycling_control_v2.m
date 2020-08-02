function modelControls = cycling_control_v2(osimModel, osimState)
    import org.opensim.modeling.*;

    %% GLOBAL VARIABLES
    % timing
    global Ts
    global n_samples

    % general variables
    global ZERO
    global refVel
    global refVel1
    global measAngle
    global measVel
    global musclesNames
    global controlActionLoad

    % general control variables
    global controlType

    global controlAction
    global controlActionHat
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
    
    %knee
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
    global flagKneeTorque
    global flagLoad
    global flagMotor
    global flagKneeStart
    global flagKneeRange
    global flagSTIM
    global flagFAT_R
    global flagFAT_L

    %% GET INFO
    modelControls = osimModel.updControls(osimState);
    thisTime = osimState.getTime();
    thisStateArray = osimModel.getStateValues(osimState);
    thisAngle = thisStateArray.get(10);
    thisVel = -(thisStateArray.get(23));
    thisKneeAngleL = thisStateArray.get(7);
    thisKneeAngleR = thisStateArray.get(4);

    %% UPDATE CONTROL (only update if sampling period has passed)
    if (thisTime - controlTime(controlIteration)) >= (Ts-.01*Ts)
        
        % add noise
        if flagSensorNoise
            sigma = deg2rad(5);
            thisAngle = thisAngle + sigma*randn(1);
            thisVel = thisVel + sigma*randn(1);
        end
        
        controlIteration = controlIteration + 1;
        controlTime(controlIteration) = thisTime;

        % print simulation evolution        
        fprintf('%d/%d: gear: %f , vel: %f, u: %f\n',controlIteration, n_samples, rad2deg(thisAngle), rad2deg(thisVel), controlAction(controlIteration-1));

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % CONTROLLER CODE HERE 
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        %%%%
        % STIM ACTION
        %%%%
        
        % define reference
        thisRefVel = speed_step_sequence(controlIteration,deg2rad(refVel1),0);

        % compute error 
        thisError = thisRefVel - thisVel;

        % compute control
        thisUhat = 0;
        if strcmp(controlType,'ILC')
            if controlIteration >= 350 % ILC PD-type
                % thisU = pedal_ilc_pdtype( thisRefPedal, thismeasAngle );
                % ILC PD-type with Q-filter
                thisU = pedal_ilc_pdtype_qfilter( thisRefVel, thisAngle );
            else % do PID
                [thisU, integralError] = cycling_pid(thisRefVel, thisAngle, controlAction(controlIteration-1), ...
                    controlError(controlIteration-1), integralError, Ts);
            end
        elseif strcmp(controlType,'PID')
            [thisU, integralError] = cycling_pid(thisRefVel, thisVel, controlAction(controlIteration-1), ...
                controlError(controlIteration-1), integralError, Ts);
            u_min = 0.7;
            if thisU < 0
                if flagSTIM == 1
                    thisU = controlAction(controlIteration-1);
                elseif flagSTIM == 2
                    thisU = u_min;
                end
            end
            
        elseif strcmp(controlType, 'fuzzy')
            [thisU] = cycling_fuzzy(thisRefVel, thisVel, controlError(controlIteration-1), ...
                 Ts);
            if thisU < 0
                thisU  = 0; 
            end 
        elseif strcmp(controlType,'Open-loop')
            if controlIteration <= 0.1/Ts % start open-loop step excitation at this instant
                thisU = 0;
            else
                thisU = cycling_openloop( flagSTIM );
            end
        end
        
        %%%%
        % PASSIVE KNEE ORTHOSES
        %%%%                
        [thisTorqueRight] = passive_orthoses(thisKneeAngleR,  flagKneeStart,flagKneeRange);
        [thisTorqueLeft] = passive_orthoses(thisKneeAngleL,  flagKneeStart,flagKneeRange);
            
        thisTorqueRight = thisTorqueRight*flagKneeTorque;
        thisTorqueLeft = thisTorqueLeft*flagKneeTorque;
        
        %%%%
        % LOAD AND MOTOR
        %%%%        
        [thisLoad] = gear_load_v2(thisVel,flagLoad,flagMotor,thisTime); % compute load and motor
                
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % MUSCLE RANGE
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % define muscle actuation
        if flagCorrection
            [ quad_l, quad_r , hams_l, hams_r, glut_l, glut_r ] = profile( thisAngle,  thisVel , 1);
        else
            [ quad_l, quad_r , hams_l, hams_r, glut_l, glut_r ] = profile( thisAngle,  thisVel , 0 );
        end
        quadRight = thisU*quad_r;
        quadLeft = thisU*quad_l;
        hamsRight = thisU*hams_r;
        hamsLeft = thisU*hams_l;
        glutRight = thisU*glut_r;
        glutLeft = thisU*glut_l;
           
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % END OF CONTROLLER CODE
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % FATIGUE HERE
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % QUAD FATIGUE
        if(flagFAT_R>0)
            if controlActionQuadRight(controlIteration) > 0
                tQUADactivatedRight = tQUADactivatedRight + Ts;
            end
            if controlActionHamsRight(controlIteration) > 0
                tHAMSactivatedRight = tHAMSactivatedRight + Ts;
            end
            if controlActionGlutRight(controlIteration) > 0
                tGLUTactivatedRight = tGLUTactivatedRight + Ts;
            end
            fQUADRight = fatigue(tQUADactivatedRight, flagFAT_R );    
            fHAMSRight = fatigue(tHAMSactivatedRight, flagFAT_R );   
            fGLUTRight = fatigue(tGLUTactivatedRight, flagFAT_R );           
        else
            fQUADRight = 0;
            fHAMSRight = 0;
            fGLUTRight = 0;
        end
        
        if(flagFAT_L>0)
            if controlActionQuadLeft(controlIteration) > 0
                tQUADactivatedLeft = tQUADactivatedLeft + Ts;
            end
            if controlActionHamsLeft(controlIteration) > 0
                tHAMSactivatedLeft = tHAMSactivatedLeft + Ts;
            end
            if controlActionGlutLeft(controlIteration) > 0
                tGLUTactivatedLeft = tGLUTactivatedLeft + Ts;
            end
            fQUADLeft = fatigue(tQUADactivatedLeft, flagFAT_L );
            fHAMSLeft = fatigue(tHAMSactivatedLeft, flagFAT_L );
            fGLUTLeft = fatigue(tGLUTactivatedLeft, flagFAT_L );
        else
            fQUADLeft = 0;
            fHAMSLeft = 0;
            fGLUTLeft = 0;
        end
        
        %% put zeros on the muscles we are not stimulating
        if (~flagMusclesQUAD)
            quadRight = 0;
            quadLeft = 0;
        else
            quadRight = quadRight-fQUADRight; %fatigue
            quadLeft = quadLeft-fQUADLeft; %fatigue
        end
        if (~flagMusclesHAMS)
            hamsRight = 0;
            hamsLeft = 0;
        else
            hamsRight = hamsRight-fHAMSRight; %fatigue
            hamsLeft = hamsLeft-fHAMSLeft; %fatigue            
        end
        if (~flagMusclesGLUT)
            glutRight = 0;
            glutLeft = 0;
        else
            glutRight = glutRight-fGLUTRight; %fatigue
            glutLeft = glutLeft-fGLUTLeft; %fatigue    
        end     
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % END OF FATIGUE 
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        refVel(controlIteration) = thisRefVel;
        controlAction(controlIteration) = thisU;
        controlActionHat(controlIteration) = thisUhat;
        controlError(controlIteration) = thisError;
        measAngle(controlIteration) = thisAngle;
        measVel(controlIteration) = thisVel;
        angleKneeL(controlIteration) = thisKneeAngleL;
        angleKneeR(controlIteration) = thisKneeAngleR;
        
        controlActionQuadRight(controlIteration) = quadRight;
        controlActionQuadLeft(controlIteration) = quadLeft;
        controlActionHamsRight(controlIteration) = hamsRight;
        controlActionHamsLeft(controlIteration) = hamsLeft;
        controlActionGlutRight(controlIteration) = glutRight;
        controlActionGlutLeft(controlIteration) = glutLeft;
        
        controlActionTorqueRight(controlIteration) = thisTorqueRight;
        controlActionTorqueLeft(controlIteration) = thisTorqueLeft;
                
        controlActionLoad(controlIteration) = thisLoad;
    else
        thisU = controlAction(controlIteration);
        
        quad_r = controlActionQuadRight(controlIteration);
        quad_l = controlActionQuadLeft(controlIteration);
        hams_r = controlActionHamsRight(controlIteration);
        hams_l = controlActionHamsLeft(controlIteration);
        glut_r = controlActionGlutRight(controlIteration);
        glut_l = controlActionGlutLeft(controlIteration);
        
        quadRight = thisU*quad_r;
        quadLeft = thisU*quad_l;
        hamsRight = thisU*hams_r;
        hamsLeft = thisU*hams_l;
        glutRight = thisU*glut_r;
        glutLeft = thisU*glut_l;
    end
    
    %% put zeros on the muscles we are not stimulating
    if (~flagMusclesQUAD)
        quadRight = 0;
        quadLeft = 0;
    end
    if (~flagMusclesHAMS)
        hamsRight = 0;
        hamsLeft = 0;
    end
    if (~flagMusclesGLUT)
        glutRight = 0;
        glutLeft = 0;
    end        
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % END OF MUSCLE ENCODING
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%
    % TORQUE ACTION
    %%%%
    
    % passive knee orthosis
    osimModel.updActuators().get('knee_torqueR').addInControls(Vector(1, controlActionTorqueRight(controlIteration)), modelControls);
    osimModel.updActuators().get('knee_torqueL').addInControls(Vector(1, controlActionTorqueLeft(controlIteration)), modelControls);
    
    % load
    osimModel.updActuators().get('gear_load').addInControls(Vector(1, controlActionLoad(controlIteration)), modelControls);

    
    % assing excitation to muscles
    for i = 1:length(musclesNames)
        if strcmp(musclesNames{i},'rect_fem_r')
            thisExcitation = Vector(1, quadRight);
        elseif strcmp(musclesNames{i},'vas_int_r')
            thisExcitation = Vector(1, quadRight);
        elseif strcmp(musclesNames{i},'rect_fem_l')
            thisExcitation = Vector(1, quadLeft);
        elseif strcmp(musclesNames{i},'vas_int_l')
            thisExcitation = Vector(1, quadLeft);
		elseif strcmp(musclesNames{i},'hamstrings_l')
            thisExcitation = Vector(1, hamsLeft);		
		elseif strcmp(musclesNames{i},'hamstrings_r')
            thisExcitation = Vector(1, hamsRight);
		elseif strcmp(musclesNames{i},'glut_max_r')
            thisExcitation = Vector(1, glutRight);		
		elseif strcmp(musclesNames{i},'glut_max_l')
            thisExcitation = Vector(1, glutLeft);
        else
            thisExcitation = Vector(1, ZERO);
        end
        % update modelControls with the new values
        if thisExcitation ~= ZERO
            osimModel.updActuators().get(musclesNames{i}).addInControls(thisExcitation, modelControls);
        end
    end
    
end