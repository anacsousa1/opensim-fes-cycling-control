function ref = speed_step_sequence( controlIteration , low_vel, high_vel )

    global Ts
    t = 35.0;
    
    % step reference
    if controlIteration*Ts < t
        ref = low_vel;
    else
        ref = high_vel;
    end   

end