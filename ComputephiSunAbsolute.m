function phiSun = ComputephiSunAbsolute (t, days, hdays, rango)
    % This function returns the position of the Sun with respect to the
    % Moon given the time. The angle is given in radians.
    % range = 1: Angles from 0 to 2pi
    % range = 0: Angles from -pi to pi
    % days is just used as a substitute of "floor" function
    % hdays is the same as days but to be used when range = 0
    % -------------------------------------------------------------------
    
    T = 29.5;               % Period
    wSun = 2*pi/T/24/3600;  % Angular speed
    if (rango == 1)         % Code for angles from 0 to 2pi
        phiSun = t*wSun - 2*pi*days;
    elseif (rango == 0)
        phiSun = t*wSun - 2*pi*(hdays-days);
    else
        error ('The range introduced is not valid');
    end
end