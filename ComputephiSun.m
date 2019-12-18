% This file contains a function that computes the angle between the Sun and
% the Rover w.r.t the Moon.

% phiSun = ComputephiSun (t, days, hdays, rango, data, do)

% Inputs:   t: time (counting since beginning of sim)
%           days: number of periods since beginning
%           hdays: number of half periods since beginning
%           rango:  0 for angle between -pi and pi
%                   1 for angles between 0 and 2*pi
%           data: eq. of orientation of the Rover
%           do: operations done to the angle
%               do.add: times that 2pi has been added
%               do.subs: times that 2pi has been substracted

% Output:   phiSun: orientation of the Sun w.r.t Rover

function phiSun = ComputephiSun (t, days, hdays, rango, data, do)
    phiSunMoon = ComputephiSunAbsolute(t, days, hdays, rango);
    phiSunRover = data(t);
    phiSun = phiSunMoon-phiSunRover;
    phiSun = phiSun - do.subs*2*pi + do.add*2*pi;
end
