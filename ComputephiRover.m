% This file contains the function for knowing the angle of the Rover w.r.t. the Moon given its equation

% function phiSun = ComputephiRover (data, t)

% Inputs:   data: equation of evolution of angle
%           t: time (since beginning of sim)

% Output:   phiSun: angle of Rover w.r.t. the Moon (rad)

function phiSun = ComputephiRover (data, t)
            phiSun = data(t);
end
