function [ quad_l, quad_r, hams_l, hams_r, glut_l, glut_r  ] = profile( angle, speed , correction)
    
    %% initial parameters
    speed = -speed;
    speed = rad2deg(speed);             
    speed_max = 500;   
    
    % with correction: cadence-based control
    if correction == 1
        correction_factor = -30;
        if speed <= speed_max
            theta_shift = (speed / speed_max) * correction_factor;
        elseif speed < 0
            theta_shift = 0;
        else
            theta_shift = correction_factor;
        end
    else
        theta_shift = 0;
    end
    
    % PROFILE: right leg <---
    right_quad_start_ang = 300;
    right_quad_range =  100;
	
	right_hams_start_ang = 70;
	right_hams_range = 100; 
    
	right_gluteus_start_ang  = 30;
	right_gluteus_range  = 70; 
    
    % PROFILE: left leg profile <---
    left_quad_start_ang = right_quad_start_ang - 180;
    left_quad_range = right_quad_range;
	
	left_hams_start_ang = right_hams_start_ang + 180;
	left_hams_range = right_hams_range;
    
	left_gluteus_start_ang  = right_gluteus_start_ang + 180;
	left_gluteus_range  = right_gluteus_range; 
    
    %% Get angle correctly    
    % gearToGround is counter-clockwise, take the absolute angle and speed
    angle = -angle;
    
    % gearToGround increases forever, wrap to 0 to 360 deg
    angle = wrapTo360(rad2deg(angle)); 
    
    %% Stimulate if it is in the stim zone: Right Leg	
	% quad
    start_ang = right_quad_start_ang - theta_shift;
    end_ang = wrapTo360(start_ang + right_quad_range);
    
    d1 = rem(angle-start_ang+180+360,360)-180;
    d2 = rem(end_ang-angle+180+360,360)-180;
    
    if (d1 >=0) && (d2 >= 0)
        quad_r = 1;
    else
        quad_r = 0;
    end
	
	% hams
    start_ang = right_hams_start_ang - theta_shift;
    end_ang = wrapTo360(start_ang + right_hams_range);
    
    d1 = rem(angle-start_ang+180+360,360)-180;
    d2 = rem(end_ang-angle+180+360,360)-180;
    
    if (d1 >=0) && (d2 >= 0)
        hams_r = 1;
    else
        hams_r = 0;
    end
	
	% glut
    start_ang = right_gluteus_start_ang - theta_shift;
    end_ang = wrapTo360(start_ang + right_gluteus_range);
    
    d1 = rem(angle-start_ang+180+360,360)-180;
    d2 = rem(end_ang-angle+180+360,360)-180;
    
    if (d1 >=0) && (d2 >= 0)
        glut_r = 1;
    else
        glut_r = 0;
    end
    
    %% Stimulate if it is in the stim zone: Left Leg	
	% quad
    start_ang = left_quad_start_ang - theta_shift;
    end_ang = wrapTo360(start_ang + left_quad_range);
    
    d1 = rem(angle-start_ang+180+360,360)-180;
    d2 = rem(end_ang-angle+180+360,360)-180;
    
    if  (d1 >=0) && (d2 >= 0)
        quad_l = 1;
    else
        quad_l = 0;
    end
	
	% hams
    start_ang = left_hams_start_ang - theta_shift;
    end_ang = wrapTo360(start_ang + left_hams_range);
    
    d1 = rem(angle-start_ang+180+360,360)-180;
    d2 = rem(end_ang-angle+180+360,360)-180;
    
    if (d1 >=0) && (d2 >= 0)
        hams_l = 1;
    else
        hams_l = 0;
    end
    
	% glut
    start_ang = left_gluteus_start_ang - theta_shift;
    end_ang = wrapTo360(start_ang + left_gluteus_range);
    
    d1 = rem(angle-start_ang+180+360,360)-180;
    d2 = rem(end_ang-angle+180+360,360)-180;
    
    if (d1 >=0) && (d2 >= 0)
        glut_l = 1;
    else
        glut_l = 0;
    end
end