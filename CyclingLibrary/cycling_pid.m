function [u, newIntegralError] = cycling_pid( refVel, thisVel, lastControlAction, lastError, lastIntegralError, Ts )

    % PID parameters
    Kp = 1.0;
    Ki = 0.45;
    Kd = 0.0;

    % compute error
    thisError = refVel - thisVel;

    % compute control action
    newIntegralError = lastIntegralError + thisError * Ts;
    u = Kp * thisError + Kd * (thisError - lastError) / Ts + ...
        Ki * newIntegralError; % PID

    % apply control saturation & anti-windup
    if u > 1
        u = 1;
        newIntegralError = lastIntegralError;
    elseif u < -1
        u = -1;
        newIntegralError = lastIntegralError;
    end

    % reset integral term for different muscle excitation
    if ~sign(lastControlAction*u)
        newIntegralError = 0;
    end

end

