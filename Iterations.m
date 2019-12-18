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
    
    Tcontrol = 9;                      % Total time of the simulation
    RovPos = @(x) 20*(x-60)./(10*sqrt(1+((x-60)./10).^2))+20;
    v = 16;
    data = ComputephiRoverEq (RovPos, v, Tcontrol, 0.25);
    
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
        
%% Useless
                    
rhist = rango;
ithist = 1;

% len = 80;


% vel = 0;
% pos = 0;


stopvel = 10;


% do.add = 0;
% do.subs = 0;

% % data = @Computesen; % Either check the values at every MPC iteration
% % (slow) or write the data in a table and read it for every iteration
% % f = @(x) sin(x);    % Trajectory of the rover

Tchange = [];
Tback = [];
% data = 0;

%%
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
%     disp(strcat('Length of the simulation: ', num2str(), 's'));
    disp('-----------------------------------------------');
    lim1 = 1;
    lim2 = 5;
    p = polyfit (data.t(lim1:lim2), data.phi(lim1:lim2), 4);
    func = ['@(x)'];
    for i = 1:size(p, 2)
        func = [func '+(' num2str(p(i)) '*x.^' num2str(size(p, 2)-i) ')'];
    end
    func(5) = [];
    func = eval (func);
    while (iter*ts <= Tcontrol)     % Simulate over the whole time 
        while iter*ts > data.t((lim2+lim1)/2) && lim2 ~= size(data.t, 2)
            lim1 = lim1+1;
            lim2 = lim2+1;
            p = polyfit (data.t(lim1:lim2), data.phi(lim1:lim2), 4);
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
        
        if(iter*ts>(days+1)*(lendays))
            days = days+1;
        end
        if(iter*ts>(hdays+1)*(lendays/2))
            hdays = hdays+1;
        end

        % 1. Correct phi such that it is inside the correct range
        [phiinit, do] = Correctphi(phiinit, rango);
        
        % 2. Choose the correct range and correct the angle
        [rango, phiinit] = SetRango (phiinit, rango, lim);
        [phiinit, ~] = Correctphi(phiinit, rango);
        
        % 3. Check how to compute phiSun in the correct range
        do.subs = 0;
        do.add = 0;
        phitemp = ComputephiSun(iter*ts, iter, days, hdays, rango, func, do);
        [phitemp, do] = Correctphi(phitemp, rango);
        
        % 4. Check how to compute the efficiency correctly
        flageff = SetFlageff(phiinit, phitemp);
        
        % 5. Check if we have to stop at some point
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
        
        % 6. Call the controller in case it is on to compute the next input
        if ~stopflag % The controller is on
            MyMotorNoInductance_wchanges;
            uhist = [uhist out.CONTROLS(2,2)];      % Save the first value of the input
            delete MyMotorNoInductance.cpp
            delete MyMotorNoInductance_RUN.m
            delete MyMotorNoInductance_RUN.mexw64
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
        if isempty (effhist)
            effhist = [effhist efftmp];
        end
        
        % 7. Simulate the system
        sim_input.u = uhist(end);
        for j = 0:Nsim-1
            sim_input.od = efftmp;
            sim_input.x = [whist(end) phihist(end) echist(end) ephist(end) ihist(end)]';
            state = integrate_vdp(sim_input);
            state.value(2) = Correctphi(state.value(2), rango);
            [rango, state.value(2)] = SetRango (state.value(2), rango, lim);
            [state.value(2), ~] = Correctphi(state.value(2), rango);
            do.subs = 0;
            do.add = 0;
            phitemp = ComputephiSun((iter*ts)+j*tsim, iter, days, hdays, rango, func, do);
            [phitemp, do] = Correctphi(phitemp, rango);
            flageff = SetFlageff(state.value(2), phitemp);
            efftmp = ComputeEff (state.value(2), phitemp, flageff);
            if (state.value(3)<echist(end))
                state.value(3) = echist(end);
            end
            if (state.value(4)<ephist(end))
                state.value(4) = ephist(end);
            end
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
        
        
        draw;
        % Check if an adjustment is needed
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

        % Plot the results of this iteration
        
        
        pause(paus)
        iter = iter+1;
%         if (rem(iter*ts, 20) ==0)
%             cd 10DecLongSimulation
%             save(strcat('T_', num2str(iter*ts)));
%             cd ..
%         end
    end
    Graphs;
% %%
% % for i = 1+1:2
% %     if (i==1)
% %         wcons = 1;
% %         wprod = 100;
% %     elseif (i ==2)
% %         wcons = 1;
% %         wprod = 1;
% %     else
% %         wcons = 100;
% %         wprod = 1;
% %     end
% %     for iter = 1:len
%     flag = 1;
%     while(flag)
% %         if (rango == 0)
% %             while (phiinit >= 180*2*pi/360)
% %                 phiinit = phiinit-2*pi;
% %             end
% %             while (phiinit < -180*2*pi/360)
% %                 phiinit = phiinit+2*pi;
% %             end
% %         elseif rango ==1
% %             while (phiinit >= 360*2*pi/360)
% %                 phiinit = phiinit-2*pi;
% %             end
% %             while (phiinit < 0)
% %                 phiinit = phiinit+2*pi;
% %             end
% %         end
% %         [phiinit, ~] = Correctphi(phiinit, rango);
% %         % Set the range in which the angles are working
% %         if (rango == 0 && abs (phiinit) > (180-lim)*2*pi/360)
% %             rango = 1;
% %             while (phiinit < 0)
% %                 phiinit = phiinit+360*2*pi/360;
% %             end
% %             disp('-----------------------------------------------');
% %             disp('The range has changed to 0:2pi');
% %             disp('-----------------------------------------------');
% %             rhist = [rhist rango];
% %             ithist = [ithist iter];
% %             beep
% %         elseif (rango == 1 && (phiinit < lim*2*pi/360 || phiinit> (360-lim)*2*pi/360))
% %             rango = 0;
% %             while (phiinit > 180*2*pi/360)
% %                 phiinit = phiinit -360*2*pi/360;
% %             end
% %             disp('-----------------------------------------------');
% %             disp('The range has changed to -pi:pi');
% %             disp('-----------------------------------------------');
% %             rhist = [rhist rango];
% %             ithist = [ithist iter];
% %             beep
% %         end
% %         [rango, phiinit] = SetRango (phiinit, rango, lim);
% %         
% %         % Check the difference of angles
% %         do.subs = 0;
% %         do.add = 0;
% %         phitemp = ComputephiSun((iter-1)*ts, iter, days, hdays, rango, data, do);
% %         if (rango == 0)
% %             % Need to make sure that phitemp is between -pi and pi
% %             while (phitemp >= 180*2*pi/360)
% %                 do.subs = do.subs+1;
% %                 phitemp = phitemp-2*pi;
% %             end
% %             while (phitemp < -180*2*pi/360)
% %                 do.add = do.add+1;
% %                 phitemp = phitemp+2*pi;
% %             end
% %         elseif (rango == 1)
% %             % Need to make sure that phitemp is between 0 and 2*pi
% %             while (phitemp >= 360*2*pi/360)
% %                 do.subs = do.subs+1;
% %                 phitemp = phitemp-2*pi;
% %             end
% %             while (phitemp < 0)
% %                 do.add = do.add+1;
% %                 phitemp = phitemp+2*pi;
% %             end
% %         else
% %             error ('Range has a wrong value');
% %         end
%         
% %         phitemp = ComputephiSun((iter-1)*ts, iter, days, hdays, rango, data, do);
% %         [phitemp, do] = Correctphi(phitemp, rango);
% %             
% %             
% %         if (abs(phiinit-phitemp)>181*2*pi/360)
% %             if (phiinit-phitemp)>0
% %                 flageff = 2;
% %             else
% %                 flageff = 1;
% %             end
% %         elseif (abs(phiinit-phitemp)<=180*2*pi/360)
% %             flageff = 0;
% % %         else
% % %             flageff = 0
% %         end
% %         
% %         if (abs(phiinit-phitemp)>360*2*pi/360)
% %             warning ('There is something wrong with the angles. The difference is big, the efficiency might be wrongly calculated');
% %             beep
% %         end
%         
% %         1. Correct phi such that it is inside the correct range
%         [phiinit, ~] = Correctphi(phiinit, rango);
% %         2. Choose the correct range
%         [rango, phiinit] = SetRango (phiinit, rango, lim);
% %         3. Check how to compute phiSun in the correct range
%         do.add = 0;
%         do.subs = 0;
%         phitemp = ComputephiSun((iter-1)*ts, iter, days, hdays, rango, data, do);
%         [phitemp, do] = Correctphi(phitemp, rango);
% %         4. Check how to compute the efficiency correctly
%         flageff = SetFlageff(phiinit, phitemp);
%         
% %         if iter >1
% %             flag = 0;
% %             for j=1:size(out.STATES(:,1))
% %                 if (size(whist, 2)>=stopvel+1)
% %                     if(abs(whist(end)-whist(end-stopvel)) >= 1e-2)
% % %                 if (abs(out.STATES(j,2))>=1e-2)
% %                         flag = 1;
% %                         break;
% %                     end
% %                 else
% %                     flag = 1;
% %                 end
% %             end
% %         end
%         flag = 1; % Bucle infinito
%         
%         if ~stopflag
%             MyMotorNoInductance_wchanges;
%             uhist = [uhist out.CONTROLS(1,2)];
%             delete MyMotorNoInductance.cpp
% %         delete MyMotorNoInductance_RUN.exp
% %         delete MyMotorNoInductance_RUN.lib
%             delete MyMotorNoInductance_RUN.m
%             delete MyMotorNoInductance_RUN.mexw64
% %         delete MyMotorNoInductance_RUN.mexw64.manifest
% 
%             if (~isempty (ihist))
%                 efftemp = ComputeEff(phiinit, phitemp, flageff);
%                 sim_input.od = efftemp;
%                 sim_input.u = uhist(end);
%                 sim_input.x = [whist(end) phihist(end) echist(end) ephist(end) ihist(end)]';
%             else
%                 efftemp = ComputeEff(0, phitemp, flageff);
%                 sim_input.od = efftemp;
%                 sim_input.u = uhist(end);
%                 sim_input.x = [whist(end) phihist(end) echist(end) ephist(end) 0]';
%             end
%             state = integrate_vdp(sim_input);
%             
%             if (state.value(3)<echist(end))
%                 state.value(3) = echist(end);
%             end
%             if (state.value(4)<ephist(end))
%                 state.value(4) = ephist(end);
%             end
% %             if (abs(state.value(1))<0.5)
% %                 state.value(1) = 0;
% %             end
% %             if (rango == 0)
% %                 while (phiinit >= 180*2*pi/360)
% %                     phiinit = phiinit-2*pi;
% %                 end
% %                 while (phiinit < -180*2*pi/360)
% %                     phiinit = phiinit+2*pi;
% %                 end
% %             elseif rango ==1
% %                 while (phiinit >= 360*2*pi/360)
% %                     phiinit = phiinit-2*pi;
% %                 end
% %                 while (phiinit < 0)
% %                     phiinit = phiinit+2*pi;
% %                 end
% %             end
%             whist = [whist state.value(1)];
%             phihist = [phihist state.value(2)];
%             echist = [echist state.value(3)];
%             ephist = [ephist state.value(4)];
%             ihist = [ihist state.value(5)];
%             
%             phiinit = state.value(2);
%             Eprodinit = state.value(4);
%             Econsinit = state.value(3);
%             winit = state.value(1);
%             
% %             phihist = [phihist out.STATES(1,3)];
% %             whist = [whist out.STATES(1,2)];
% %             if (~isempty (ephist) && ephist(end)>out.STATES(1,5))
% %                 out.STATES(1,5) = ephist(end);
% %             end
% %             ephist = [ephist max(0, out.STATES(1,5))];
% %             if (~isempty (echist) && echist(end)>out.STATES(1,4))
% %                 out.STATES(1,4) = echist(end);
% %             end
% %             echist = [echist out.STATES(1,4)];
% %             
% %             ihist = [ihist out.ALGEBRAICSTATES(1,2)];
%             effhist = [effhist ComputeEff(out.STATES(1, 3),ComputephiSun(ts*(iter), iter, days, hdays, rango, data, do), flageff)];
%             phist = [phist out.CONTROLS(1,2).*ihist(end)];
% % %             vel = out.STATES(1,2);
% % %             pos = phihist(end);
% %             phiinit = out.STATES(2,3);
% %             Eprodinit = max(0, out.STATES(2,5));
% %             Econsinit = out.STATES(2,4);
% %             winit = out.STATES(2,2);
%         else
%             uhist = [uhist 0];
%             if (~isempty (ihist))
%                 efftemp = ComputeEff(phiinit, phitemp, flageff);
%                 sim_input.od = efftemp;
%                 sim_input.u = 0;
%                 sim_input.x = [whist(end) phihist(end) echist(end) ephist(end) ihist(end)]';
%             else
%                 efftemp = ComputeEff(0, phitemp, flageff);
%                 sim_input.od = efftemp;
%                 sim_input.u = 0;
%                 sim_input.x = [winit phiinit Econsinit Eprodinit 0]';
%             end
%             state = integrate_vdp(sim_input);
% %             if (abs(state.value(1))<0.5)
% %                 state.value(1) = 0;
% %             end
%             
%             if (state.value(3)<echist(end))
%                 state.value(3) = echist(end);
%             end
%             if (state.value(4)<ephist(end))
%                 state.value(4) = ephist(end);
%             end
% %             if (rango == 0)
% %                 while (phiinit >= 180*2*pi/360)
% %                     phiinit = phiinit-2*pi;
% %                 end
% %                 while (phiinit < -180*2*pi/360)
% %                     phiinit = phiinit+2*pi;
% %                 end
% %             elseif rango ==1
% %                 while (phiinit >= 360*2*pi/360)
% %                     phiinit = phiinit-2*pi;
% %                 end
% %                 while (phiinit < 0)
% %                     phiinit = phiinit+2*pi;
% %                 end
% %             end
%             whist = [whist state.value(1)];
%             phihist = [phihist state.value(2)];
%             echist = [echist state.value(3)];
%             ephist = [ephist state.value(4)];
%             ihist = [ihist state.value(5)];
%             
%             phiinit = state.value(2);
%             Eprodinit = state.value(4);
%             Econsinit = state.value(3);
%             winit = state.value(1);
%             effhist = [effhist ComputeEff(phihist(end),ComputephiSun(ts*(iter), iter, days, hdays, rango, data, do), flageff)];
%             phist = [phist uhist(end)*ihist(end)];
% %             phihist = [phihist out.STATES(1,3)];
% %             whist = [whist 0];
% %             effhist = [effhist ComputeEff(out.STATES(1, 3),ComputephiSun(ts*(iter), iter, days, hdays, rango, data, do), flageff)];
% %             if (~isempty (effhist) && effhist(end) < 0)
% %                 ephist = [ephist ephist(end)];
% %             else
% %                 ephist = [ephist ephist(end)+effhist(end)*ts*Pmax];
% %             end
% %             echist = [echist echist(end)];
% %             uhist = [uhist 0];
% %             ihist = [ihist 0];
% %             phist = [phist 0];
% %             if (~isempty(phihist))
% %                 phiinit = phihist(end);
% %             else
% %                 phiinit = 0;
% %             end
% %             if (~isempty(ephist))
% %                 Eprodinit = ephist(end);
% %             else
% %                 Eprodinit = 0;
% %             end
% %             if (~isempty(echist))
% %                 Econsinit = echist(end);
% %             else
% %                 Econsinit = 0;
% %             end
% %             if (~isempty(whist))
% %                 winit = whist(end);
% %             else
% %                 winit = 0;
% %             end
% %             vel = out.STATES(1,2);
% %             pos = phihist(end);
%         end
%         
%         
% 
% %         if(iter ==2 || iter ==3 || iter==11 || iter==31)
% %             saveas(fig, strcat('Fixed_Wcons_',num2str(wcons),'_Wprod_',num2str(wprod),'_iteration_',num2str(iter-1),'.eps'), 'epsc');
% %             saveas(fig, strcat('Fixed_Wcons_',num2str(wcons),'_Wprod_',num2str(wprod),'_iteration_',num2str(iter-1),'.fig'));
% %             fig = figure();
% %             beep
% %            
% %         end
% 
%         clf;
%         if (iter >1)
%             subplot(4,3,1)
%             plot(out.STATES(:,1), out.ALGEBRAICSTATES(:,2), 'r', 0:ts:(size(ihist, 2)-1)*ts, ihist, 'b')
%             title('Current');
%     %         hold on
%     %         plot(out.STATES(:,1), out.STATES(:,2))
%     %         hold off
%     %         xlim ([0, len*ts]);
% 
%             subplot(4,3,10:11)
%             plot(out.STATES(:,1), out.STATES(:,3), 'r', 0:ts:(size(phihist, 2)-1)*ts, phihist, 'b')
%             title('Angular position');
%             hold;
%             time = 0:ts:80*ts;
%             phisunplot = ComputephiSun(0:ts:80*ts, iter, days, hdays, rango, data, do);
%             plot(time, phisunplot, 'k', ts*(iter:iter+N),ComputephiSun(ts*(iter):ts:N*ts+ts*(iter),iter, days, hdays, rango, data, do),'g')
%             legend('PhiPanel','Implemented', 'PhiSun', 'PhiSunHorizon', 'Location', 'northwest');
%             hold off;
%     %         xlim ([0, len*ts]);
% 
%             subplot(4,3,2)
%             plot(out.STATES(:,1), out.CONTROLS(:,2).*out.ALGEBRAICSTATES(:,2), 'r', 0:ts:(size(phist, 2)-1)*ts, phist, 'b')
%             title('Power');
%     %         xlim ([0, len*ts]);
% 
%             subplot(4,3,4)
%             stairs(out.STATES(:,1), out.CONTROLS(:,2), 'r')
%             hold on;
%             if size(uhist, 2)>=1
%                 stairs(0:ts:(size(uhist, 2)-1)*ts, uhist, 'b')
%             end
%             title('Control');
%             hold off;
%     %         xlim ([0, len*ts]);
% 
%             subplot(4,3,[3 6])
%             plot(out.STATES(:,1), out.STATES(:,4), 'r', 0:ts:(size(echist, 2)-1)*ts, echist, 'b')
%             title('Consumed Energy');
%     %         xlim ([0, len*ts]);
% 
%             subplot(4,3,[9 12])
%             plot(out.STATES(:,1), max(0,out.STATES(:,5)), 'r', 0:ts:(size(ephist, 2)-1)*ts, ephist, 'b')
%             title('Generated Energy');
%     %         xlim ([0, len*ts]);
% 
%             subplot(4,3,5)
%             stairs(out.STATES(:,1), ComputeEff(out.STATES(:, 3)',ComputephiSun(ts*(iter):ts:N*ts+ts*(iter),iter, days, hdays, rango, data, do), flageff), 'r')
%             hold on
%             stairs(0:ts:(size(effhist, 2)-1)*ts, effhist, 'b')
%             hold off
%             title('Efficiency');
% 
%             subplot(4, 3, 7:8)
%             plot(out.STATES(:,1), out.STATES(:,2), 'r', 0:ts:(size(whist, 2)-1)*ts, whist, 'b')
%             title('Speed');
%         end
% 
% %         if((ts*iter)>=1.6 && flag1 ==0)
% %             flag1 =1;
% %         end
% 
% %         
% %         phiinit = out.STATES(2,3);
% %         Eprodinit = max(0, out.STATES(2,5));
% %         Econsinit = out.STATES(2,4);
% %         winit = out.STATES(2,2);
%         pause(paus);
% %         if iter ==9
% %             pause
% %         end
%         if iter == 160
%             cd Test
%             saveas(fig, strcat('Test2_N15_','iteration_',num2str(iter-1),'.eps'), 'epsc');
%             saveas(fig, strcat('Test2_N15_','iteration_',num2str(iter-1),'.fig'));
%             save('Test2.mat');
%             cd ..
%             beep
%         end
%         iter = iter+1;
%         
%         
% %         if (((iter-2)*ts) >= 1-0.005 && ((iter-2)*ts) <= 1+0.005)
% %             disp('-----------------------------------------------');
% %             disp('Disturbance: the panels are suddenly in phi = 0 degrees');
% %             disp('-----------------------------------------------');
% %             beep
% %             phiinit = 0;%-10*2*pi/360;
% %             phihist(end) = phiinit;
% %         end
% %         if (((iter-2)*ts) >= 0-0.005 && ((iter-2)*ts) <= 0+0.005)
% %             stopflag = 1;
% %             disp('-----------------------------------------------');
% %             disp('The vehicle is in a dark place');
% %             disp('-----------------------------------------------');
% %             beep
% %         elseif (((iter-2)*ts) >= 1.4-0.005 && ((iter-2)*ts) <= 1.4+0.005)
% %             stopflag = 0;
% %             disp('-----------------------------------------------');
% %             disp('I can obtain energy again');
% %             disp('-----------------------------------------------');
% %             beep
% %             itreturn = iter;
% %         end
%         
%         if check ==1
%             count = count +1;
%             if count == 5
%                 stopflag = 0;
%                 check = 0;
%                 count = 0;
%                 itreturn = iter;
%             end
%         end
%         if (stopflag == 0 && -itreturn+iter > 40)
%             if(abs(effhist(end)-effhist(end-40))< 0.005 && effhist(end)<0.01)
%                 stopflag = 1;
%                 check = 1;
%                 disp('-----------------------------------------------');
%                 disp('Adjustment stop');
%                 disp('-----------------------------------------------');
%                 beep
%             end
%         end
%         if effhist(end)>effth
%             itreturn = iter;
%         end
%         
%         
% %         if(iter*ts*24>(days+1)*(lendays))
% %             days = days+1;
% %             if range == 1
% %                 phiinit = phiinit-2*pi;
% %             end
% %         end
% %         if(iter*ts*24>(hdays+1)*(lendays/2))
% %             hdays = hdays+1;
% %             if range == 0 && rem(hdays,2)
% %                 phiinit = phiinit-2*pi;
% %             end
% %         end
%         
% 
%     end
% %      iter
% %     beep
% %     i = i+1;
% %     saveas(fig, strcat('N15_Fixed_Wcons_',num2str(wcons),'_Wprod_',num2str(wprod),'_iteration_',num2str(iter-1),'.eps'), 'epsc');
% %     saveas(fig, strcat('N15_Fixed_Wcons_',num2str(wcons),'_Wprod_',num2str(wprod),'_iteration_',num2str(iter-1),'.fig'));
% %     cd NewEff
% %     saveas(fig, strcat('N15_Fixed_','iteration_',num2str(iter),'.eps'), 'epsc');
% %     saveas(fig, strcat('N15_Fixed_','iteration_',num2str(iter),'.fig'));
% %     cd ..
%     
% %     fig = figure();
% %     phihist = [];
% %     ephist = [];
% %     echist = [];
% %     uhist = [];
% %     ihist = [];
% %     phist = [];
% %     phiinit = 0;
% %     winit = 0;
% %     pos = 0;
% %     vel = 0;
% %     Econsinit = 0;
% %     Eprodinit = 0;
% %     iter = 1;
% % end
% 
% 
