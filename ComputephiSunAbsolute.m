% This file contains a function that computes the position of the Sun
% w.r.t. the Moon at a given time

% phiSun = ComputephiSunAbsolute (t, days, hdays, rango)

% Inputs:   t: time (counting since beginning of sim)
%           days: number of periods since beginning
%           hdays: number of half periods since beginning
%           rango:  0 for angle between -pi and pi
%                   1 for angles between 0 and 2*pi

% Output:   phiSun: orientation of the Sun w.r.t the Moon

function phiSun = ComputephiSunAbsolute (t, days, hdays, rango)    
    T = 29.5;               % Period
    wSun = 2*pi/T/24/3600;  % Angular speed
    if (rango == 1)
        phiSun = t*wSun - 2*pi*days;
    elseif (rango == 0)
        phiSun = t*wSun - 2*pi*(hdays-days);
    else
        error ('The range introduced is not valid');
    end
end
