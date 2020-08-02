function [torque_gear]=gear_load_v2(thisVel, L, M, t)
    vel_init = 200;
    t_init = 1;
%     t_init = 5;

    if (thisVel <= 0)
        l = 0;
    else
        l = -L; %% load against movement
    end
    
    if M~=0
        if (thisVel < deg2rad(vel_init) && t < t_init)
            m = M;  %% friction-drive wheel accelerating system (Chaichaowarat2018)
            l = 0;
        else            
            m = 0; 
        end
    else
        m = 0;
    end
    
    torque_gear = m + l;
end


%salvar valor motor