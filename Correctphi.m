% This file contains a function that modifies the input angle so that it is
% within the range.

% [phi, do] = Correctphi(phi,range)

% Inputs:   phi: angle that potentially needs to be modified (in rad)
%           range:  0 for angle between -pi and pi
%                   1 for angles between 0 and 2*pi

% Outputs:  phi: the modified angle
%           do: the operations done to the angle
%               do.add: number of times that 2pi has been added
%               do.subs: number of times that 2pi has been substracted

function [phi, do] = Correctphi(phi, range)
    do.add = 0;
    do.subs = 0;
    if (range == 0)
        while (phi >= 180*2*pi/360)
            phi = phi-2*pi;
            do.subs = do.subs+1;
        end
        while (phi < -180*2*pi/360)
            phi = phi+2*pi;
            do.add = do.add+1;
        end
    elseif range ==1
        while (phi >= 360*2*pi/360)
            phi = phi-2*pi;
            do.subs = do.subs+1;
        end
        while (phi < 0)
            phi = phi+2*pi;
            do.add = do.add+1;
        end
    else
        error ('The range is not correct. Set a 0 or 1 in variable "rango"');
    end
end
