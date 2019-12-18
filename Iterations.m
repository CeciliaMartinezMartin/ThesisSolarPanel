% This is the main code used for the simulations. This code runs in a loop
% the optimization problem, simulating the system and implementing the
% first solution.

clear all
close all
clc

SIMEXPORT = 0;

%% Motor parameters
    Pmax 	= 153;                  % max power (W)
    Kemf 	= 1/(1700*2*pi/60);     % speed constant (V/(rad/(s)))
    R       = 2.79;                 % motor terminal resistance (ohm)
    L 	    = 0.0857e-3;            % motor terminal inductance (H)
    Km      = 5.62/1000;            % torque constant (Nm/A)
    Jr    	= 1e-7;                 % rotor inertia (kgm2)
    Jg    	= 0.161e-7;             % gear inertia (kgm2)
    Jl    	= 0.0387;               % load inertia (kgm2) (provisional value)
    n 	    = 138;                  % reduction
    imax    = 0.72;                 % maximum continuous current (A)
    umax    = 6;                    % maximum input voltage (V)
    wmax    = 6770*2*pi/60;         % maximum speed (rad/s)
    b       = Km*Kemf/R;            % viscuous friction (Nm/(rad/s))
    
%% Simulation parameters
    N   = 15;       % Length of the prediction horizon
    N2  = N;        % Length of the back-up prediction horizon
    Ninit = N;
    Non     = 0;
    Nmax    = 20;   % Maximum allowed length of the prediction horizon
    ts  = 0.05;     % Sampling time
    T   = N*ts;     % Prediction horizon time
    paus = 0.2;     % Pause time between iterations
    tsim = 0.001;
    Nsim = ts/tsim;
    
    Tcontrol = 700;       % Total time of the simulation
    RovPos = @(x) 20*(x-60)./(10*sqrt(1+((x-60)./10).^2))+20;
    v = 0.2;
    data = ComputephiRoverEq (RovPos, v, Tcontrol, 20);
    
    wprod = 1;  % Weight for the produced energy
    wcons = 1;  % Weight for the consumed energy

%% Initialization of the simulation
    phiinit     = 0;    % Initial angle
    winit       = 0;    % Initial angular speed
    Econsinit   = 0;    % Initial consumed energy
    Eprodinit   = 0;    % Initial produced energy
    iter        = 0;    % Number of iteration of the MPC
    iinit       = 0;
    pinit       = 0;

    % Plotting variables initialization
    phihist = [phiinit];
    ephist = [Eprodinit];
    echist = [Econsinit];
    uhist = [];
    ihist = [iinit];
    effhist = [];
    whist = [winit];
    phist = [pinit];
    fig = figure();
    
    phicontrol      = ones(1, N)*phiinit;
    wcontrol        = ones(1, N)*winit;
    eccontrol       = ones(1, N)*Econsinit;
    epcontrol       = ones(1, N)*Eprodinit;
    icontrol        = ones(1, N)*iinit;
    ucontrol        = ones(1, N)*0;
    timecontrol     = ones(1, N)*0;
    
    
%% Angle management variables
    T       = 29.5;             % Period of the Sun (days)
    days    = 0;                % Number of day in which  we are in
    lendays = T*24*3600;        % Time until reset Sun cycle (s)
    hdays   = 0;                % Number of "half days"
        
    do.add  = 0;    % Compute the angle within the limits
    do.subs = 0;    % Compute the angle within the angles
    flageff = 0;    % Flag for the efficiency computation
    
    rango   = 0;      % 0 -> -pi to pi
                      % 1 -> 0 to 2*pi
    lim     = 30;     % Limits for the angle range change (degrees)
                    
%% Stop management variables
    stopflag    = 0;        % 0 -> use the controller
                            % 1 -> don't use the controller
                            
    check       = 0;        % 0 -> adjustment stop off
                            % 1 -> adjustment stop on
    count       = 0;        % Number of iterations since the adjustment stop began
    efftime     = 0.5;      % Time to stop the adjustment pause
    ittime      = 0.8;      % Time to decide if we are parallel
    itreturn    = iter;     % Moment in which the efficiency is less than effth
    effth       = 0.05;     % Threshold for the efficiency
    
    dostop      = 0;        % Flag to turn on stopping (0 stop off, 1 stop on)
    stoptime    = [10 60];        % Time when the vehicle gets inside a dark area
    gotime      = [1.2 70];      % Time when there is light again
    stopping    = 0;
    
%% Disturbance
    dist        = 0;        % 0 -> disturbance off
                            % 1 -> disturbance on 
    tdist       = 2;        % Time when the disturbance happens
    phidist     = -180;        % Angle to which the system is put (degrees)

%% Create and Export the simulation (only necessary once)
if SIMEXPORT
    disp('-----------------------------------------------');
    disp('Compile the simulator');
    disp('-----------------------------------------------');
    Simulation_motor;
    disp('-----------------------------------------------');
    disp('Simulator compiled');
    disp('-----------------------------------------------');
else
    disp('-----------------------------------------------');
    disp('The simulator has no been recompiled');
    disp('-----------------------------------------------');
end

%% Control loop
    disp('-----------------------------------------------');
    disp('Start the control loop');
    disp('Parameters:');
    disp(strcat('Length of the simulation: ', num2str(Tcontrol), ' s'));
    disp(strcat('Length of the prediction horizon: ', num2str(N), ' iterations'));
    disp(strcat('Sampling time: ', num2str(ts), ' s'));
    traj = func2str(RovPos);
    disp(strcat('Trajectory of the vehicle: y = ', traj(5:end)));
    disp('-----------------------------------------------');
    % Calculation of first polynomial for angle of the Rover w.r.t. Moon
    lim1 = 1;
    lim2 = 5;
    p = polyfit (data.t(lim1:lim2), data.phi(lim1:lim2), lim2-lim1);
    func = ['@(x)'];
    for i = 1:size(p, 2)
        func = [func '+(' num2str(p(i)) '*x.^' num2str(size(p, 2)-i) ')'];
    end
    func(5) = [];
    func = eval (func);
    
    % Beginning of control loop
    while (iter*ts <= Tcontrol)     % Simulate over the whole time 
        % Calculate the new polynomial for the angle, if needed
        while iter*ts > data.t((lim2+lim1)/2) && lim2 ~= size(data.t, 2)
            lim1 = lim1+1;
            lim2 = lim2+1;
            p = polyfit (data.t(lim1:lim2), data.phi(lim1:lim2), lim2-lim1);
            func = ['@(x)'];
            for i = 1:size(p, 2)
                func = [func '+(' num2str(p(i)) '*x.^' num2str(size(p, 2)-i) ')'];
            end
            func(5) = [];
            func = eval (func);
            disp('-------------------------------------')
            disp('I just changed the interval');
            disp('-------------------------------------')
        end
        
        % Check how many periods have passed since the beginning of the program
        if(iter*ts>(days+1)*(lendays))
            days = days+1;
        end
        if(iter*ts>(hdays+1)*(lendays/2))
            hdays = hdays+1;
        end

        % Correct phi such that it is inside the correct range
        [phiinit, do] = Correctphi(phiinit, rango);
        
        % Choose the correct range and correct the angle
        [rango, phiinit] = SetRango (phiinit, rango, lim);
        [phiinit, ~] = Correctphi(phiinit, rango);
        
        % Check how to compute phiSun in the correct range
        do.subs = 0;
        do.add = 0;
        phitemp = ComputephiSun(iter*ts, days, hdays, rango, func, do);
        [phitemp, do] = Correctphi(phitemp, rango);
        
        % Check how to compute the efficiency correctly
        flageff = SetFlageff(phiinit, phitemp);
        
        % Check if we have to stop at some point
        if (dostop)
            if(~isempty(stoptime))
                if iter*ts >= stoptime(1) - 0.0001 && iter*ts <= stoptime(1) + 0.0001
                    stopflag = 1;
                    disp('-----------------------------------------------');
                    disp('Dark zone or Battery full');
                    disp('-----------------------------------------------');
                    stopping = 1;
                    beep
                    stoptime(1) = [];
                end
            end
            if(~isempty(gotime))
                if iter*ts >= gotime(1) - 0.0001 && iter*ts <= gotime(1) + 0.0001
                    stopflag = 0;
                    disp('-----------------------------------------------');
                    disp('Can obtain energy again');
                    disp('-----------------------------------------------');
                    beep
                    stopping = 0;
                    itreturn = iter;    % Start checking from this moment the parallel trajectory case
                    gotime(1) = [];
                end
            end
        end
        
        % Call the controller (in case it is on) to compute the next input
        if ~stopflag % The controller is on
            MyMotorNoInductance_wchanges;
            uhist = [uhist out.CONTROLS(2,2)];      % Save the first value of the input
            % Delete created files (avoid errors on other iterations)
            delete MyMotorNoInductance.cpp
            delete MyMotorNoInductance_RUN.m
            delete MyMotorNoInductance_RUN.mexw64
            % Save predicted values (just for plotting)
            timecontrol     = out.STATES(:,1);
            icontrol        = out.ALGEBRAICSTATES(:,2);
            phicontrol      = out.STATES(:,3);
            ucontrol        = out.CONTROLS(:,2);
            eccontrol       = out.STATES(:,4);
            epcontrol       = out.STATES(:,5);
            wcontrol        = out.STATES(:,2);
        else % The controller is off
            uhist = [uhist 0];      % Set the next input to 0
        end
        efftmp = ComputeEff (phiinit, phitemp, flageff);
        if isempty (effhist) % Get the initial value of the efficiency
            effhist = [effhist efftmp];
        end
        
        % Simulate the system
        sim_input.u = uhist(end);
        for j = 0:Nsim-1 % Simulate for the number of iterations corresponding to a total time of ts
            sim_input.od = efftmp;
            sim_input.x = [whist(end) phihist(end) echist(end) ephist(end) ihist(end)]';
            state = integrate_vdp(sim_input); % Call the simulator
            state.value(2) = Correctphi(state.value(2), rango); % Put the angle in the correct range
            [rango, state.value(2)] = SetRango (state.value(2), rango, lim); % See the range it should be working on
            [state.value(2), ~] = Correctphi(state.value(2), rango); % Correct the angle
            do.subs = 0;
            do.add = 0;
            phitemp = ComputephiSun((iter*ts)+j*tsim, days, hdays, rango, func, do); % Check the value of phiSun
            [phitemp, do] = Correctphi(phitemp, rango); % Correct and check the operations done
            flageff = SetFlageff(state.value(2), phitemp); % See how to compute the efficiency
            efftmp = ComputeEff (state.value(2), phitemp, flageff); % Compute the efficiency
            if (state.value(3)<echist(end)) % The consumed energy must always grow or stay
                state.value(3) = echist(end);
            end
            if (state.value(4)<ephist(end)) % The obtained energy must always grow or stay
                state.value(4) = ephist(end);
            end
            % Save variables for plotting
            whist = [whist state.value(1)];
            phihist = [phihist state.value(2)];
            echist = [echist state.value(3)];
            ihist = [ihist state.value(5)];
            ephist = [ephist state.value(4)];
            effhist = [effhist efftmp];
            phist = [phist uhist(end)*ihist(end)];
        end
        
        % Set the initial conditions for the next iteration
        phiinit = phihist(end);
        Eprodinit = ephist(end);
        Econsinit = echist(end);
        winit = whist(end);
        iinit = ihist(end);
     
        draw; % Plot the results for the current MPC iteration
        % Check if an adjustment is needed (parallel trajectory solution)
        if check ==1
            count = count +1;
            if stopping == 0        % Only do this if we are not in stop time on purpose
                if count*ts == efftime
                    if (Ninit == N)
                        stopflag = 0;
                        check = 0;
                        count = 0;
                        itreturn = iter;
                        disp('-----------------------------------------------');
                        disp('Adjustment stop finished');
                        disp('-----------------------------------------------');
                    else
                        Non = 0;
                        check = 0;
                        count = 0;
                        if(abs(effhist(end)-effhist(end-40))> effth || effhist(end)>effth)
                            itreturn = iter;
                            disp('-----------------------------------------------');
                            disp('N has the initial value again');
                            disp('-----------------------------------------------');
                            Tback = [Tback iter*ts];
                            N = Ninit;
                        end
                    end
                end
            end
        end
        if (stopflag == 0 && (-itreturn+iter)*ts > ittime && Non == 0)
            if(abs(effhist(end)-effhist(end-40))< effth && effhist(end)<effth)
                if ( N2 >= Nmax)
                    N = Ninit;
                    stopflag = 1;
                    check = 1;
                    disp('-----------------------------------------------');
                    disp('Adjustment stop');
                    disp('-----------------------------------------------');
                    beep
                    N2 = N;
                else
                    N2 = N2 + 5;
                    N  = N2;
                    check = 1;
                    disp('-----------------------------------------------');
                    disp(strcat('N increased to ', num2str(N2)));
                    disp('-----------------------------------------------');
                    beep
                    Tchange = [Tchange iter*ts];
                    Non = 1;
                end
            end
        end
        if effhist(end)>effth
            itreturn = iter;
        end
        
        % Disturbance in the position
        if dist == 1
            if (((iter)*ts) >= tdist-0.0001 && ((iter)*ts) <= tdist+0.0001)
                disp('-----------------------------------------------');
                disp(strcat('Disturbance: the panels changed suddenly ', num2str(phidist), ' degrees'));
                disp('-----------------------------------------------');
                beep
                phiinit = phiinit + phidist*2*pi/360;
                phihist(end) = phiinit;
            end
        end

        pause(paus)
        iter = iter+1;
    end
    Graphs; % get final graphs
