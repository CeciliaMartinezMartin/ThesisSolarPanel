% This file contains a function that computes a sequence of angles of the
% Rover w.r.t. the Moon for several time points

% data = ComputephiRoverEq (RovPos, v, Tcontrol, tsamp, initpos)

% Inputs:   RovPos: movement of the Rover (func.handle)(y = f(x)) (m)
%           v: speed of the Rover (m/s)
%           Tcontrol: total time of simulation (s)
%           (tsamp): sampling time for the angle computation (s)
%               default: every 4m
%           (initpos): initial x coordinate of the Rover (m)
%               default: x = 0

% Output:  data:
%               data.t: time points for which an angle is calculated
%               data.phi: angle for each time point

function data = ComputephiRoverEq (RovPos, v, Tcontrol, tsamp, initpos)
    if (nargin < 5)
        initpos = 0;
    end
    if (nargin < 4)
        tsamp = 4/v;
    end
    if (nargin <3)
        error ('Missing some input arguments')
    end
    disp(strcat('Length of the simulation: ', num2str(Tcontrol), ' s'));
    disp(strcat('Speed: ', num2str(v), ' m/s'));
    traj = func2str(RovPos);
    disp(strcat('Equation: ', traj(5:end)));
    disp(strcat('Sampling time: ', num2str(tsamp), ' s'));
    disp(strcat('Initial position: ', num2str(initpos), ' m'));
    disp('Calculating phi sequence for the whole trajectory...');
    lengthTotal = v*Tcontrol;
    lengthShort = v*tsamp;
    a = initpos;
    syms b x
    fdiff = char(diff(RovPos(x)));
    i = length(fdiff);
    while (i > 1)
        if (fdiff (i) == '^' || fdiff (i) == '/' || fdiff(i) == '*')
            fant = fdiff(1:i-1);
            fdesp = fdiff (i:end);
            fdiff = [fant '.' fdesp];
        end
        i = i-1;
    end
    fdiff = eval(['@(x)' fdiff]);
    xsamp = [a];
    ts = [0];
    while (i*lengthShort < lengthTotal)
       eqn = int (sqrt((fdiff(x)^2 +1)), a, b) == lengthShort;
       bsol = vpasolve (eqn, b);
       b2 = bsol(imag(bsol) == 0);
       if (size(b2, 1) > 1 || size(b2, 2) > 1)
           error ('Which one??')
       end
       xsamp = [xsamp double(b2)];
       a = double(b2);
       ts = [ts ts(end)+tsamp];
       i = i+1;
    end
    eqn = int (sqrt((fdiff(x)^2 +1)), a, b) == lengthShort;
    bsol = vpasolve (eqn, b);
    b2 = bsol(imag(bsol) == 0);
    if (size(b2, 1) > 1 || size(b2, 2) > 1)
       error ('Which one??')
    end
    xsamp = [xsamp double(b2)];
    a = double(b2);
    ts = [ts ts(end)+tsamp];
    
    data.phi = atan(fdiff(xsamp));
    data.t = ts;
    
    disp('...DONE!');
end
