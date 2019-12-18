% This file contains a function that computes the flag to know in which
% "direction" to substract the angles.
% flageff = SetFlageff(phi1, phi2)
% Inputs:   phi1: first angle (in rad)
%           phi2: second angle (in rad)
% Output:  flageff:
%               0 when the absolute value of the difference by default is smaller than pi
%               1 when the difference is negative and bigger in absolute
%               value than pi
%               2 when the difference is positive and bigger than pi


function flageff = SetFlageff(phi1, phi2)
    if (abs(phi1-phi2)>180*2*pi/360)
        if (phi1-phi2)>0
            flageff = 2;
        else
            flageff = 1;
        end
    elseif (abs(phi1-phi2)<=180*2*pi/360)
        flageff = 0;
    end

    if (abs(phi1-phi2)>360*2*pi/360)
        warning ('There is something wrong with the angles. The difference is big, the efficiency might be wrongly calculated');
        beep
    end
end