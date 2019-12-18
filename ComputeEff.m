% This file contains a function that computes the efficiency of the panels
% given a position and the Sun's position

% eff = ComputeEff (phiPanel, phiSun, flageff)

% Inputs:   phiPanel: Orientation of the panel w.r.t the Rover (in rad)
%           phiSun: Orientation of the Sun w.r.t. the Rover (in rad)
%           flageff:
%               0 when the absolute value of the difference by default is smaller than pi
%               1 when the difference is negative and bigger in absolute
%               value than pi
%               2 when the difference is positive and bigger than pi

% Output:   eff: Efficiency of the panels (%)

function eff = ComputeEff (phiPanel, phiSun, flageff)
    if flageff == 0
        m = (phiPanel-phiSun)*360/2/pi;
    elseif flageff == 1
        m = (phiPanel-phiSun)*360/2/pi+360;
    elseif flageff == 2
        m = (phiPanel-phiSun)*360/2/pi-360;
    else
        error('Flageff has a wrong value');
    end

    eff =(-m.^4/1000000+99.37322857).*(((1./(1+exp((m-67)/10)))-(1./(1+exp((m+67)/10)))))+...
        ((-(m+360).^4/1000000+99.37322857).*((1./(1+exp(((m+360)-67)/10)))-(1./(1+exp(((m+360)+67)/10)))))+...
        ((-(m-360).^4/1000000+99.37322857).*((1./(1+exp(((m-360)-67)/10)))-(1./(1+exp(((m-360)+67)/10)))));
    parab = 1-(m/100).^2;
    parab2= 1-((m+360)/100).^2;
    parab3= 1-((m-360)/100).^2;
    aux = eff+(1-((1./(1+exp((m-60)/5)))-(1./(1+exp((m+60)/5))))-((1./(1+exp((m+180)/20))))-(1-(1./(1+exp((m-180)/20))))).*parab+...
        ((((1./(1+exp((m+180)/20)))))-(((1./(1+exp(((m+360)-60)/5)))-(1./(1+exp(((m+360)+60)/5)))))).*parab2+...
        (1-(((1./(1+exp((m-180)/20)))))-(((1./(1+exp(((m-360)-60)/5)))-(1./(1+exp(((m-360)+60)/5)))))).*parab3;
    eff = aux./100;
end
