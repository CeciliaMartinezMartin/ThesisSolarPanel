% This file contains a function that computes the range of angles in which
% the program should be working.
% [range, phi] = SetRango (phi, range, lim)
% Inputs:   phi: angle that potentially needs to be modified (in rad)
%           range: range in which we are currently working.
%                   0 for angle between -pi and pi
%                   1 for angles between 0 and 2*pi
%           lim: limits with respect to 0 and pi within which the range
%           must be changed
% Outputs:  phi: the modified angle
%           range: the new range of work
%                   0 for angle between -pi and pi
%                   1 for angles between 0 and 2*pi


function [range, phi] = SetRango (phi, range, lim)
    if (range == 0 && abs (phi) > (180-lim)*2*pi/360)
        range = 1;
        while (phi < 0)
            phi = phi+360*2*pi/360;
        end
        disp('-----------------------------------------------');
        disp('The range has changed to 0:2pi');
        disp('-----------------------------------------------');
        beep
    elseif (range == 1 && (phi < lim*2*pi/360 || phi> (360-lim)*2*pi/360))
        range = 0;
        while (phi > 180*2*pi/360)
            phi = phi -360*2*pi/360;
        end
        disp('-----------------------------------------------');
        disp('The range has changed to -pi:pi');
        disp('-----------------------------------------------');
        beep
%     else
%         error ('Something went wrong when correcting the range');
    end
end