function [torqueKnee]=passive_orthoses(thisAngle, startAngle,rangeAngle)

    torqueMax = 1;
    angle=wrapTo360(rad2deg(-thisAngle)); %Minus sign: thisAngle is counterclock wise
    
    %% knee torque
    if (angle>startAngle && angle<(startAngle+rangeAngle)*1.05)
            torqueKnee=torqueMax/(rangeAngle)*(startAngle-angle);
    else
        torqueKnee=0;
    end

end