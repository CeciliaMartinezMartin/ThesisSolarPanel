% clear;
% clc;
% iter = 1;


BEGIN_ACADO;                                
    
    acadoSet('problemname', 'MyMotorNoInductance');     %Set the name of the problem
%% Parameters
%     Pmax 	= 290;               % max power (W) (provisional value)
%     Kemf 	= 1/(1700*2*pi/60);	% speed constant (V/(rad/(s)))
%     R      = 2.79;              % Motor terminal resistance (ohm)
%     Km     = 5.62/1000;         % torque constant (Nm/A)
%     Jr    	= 1e-7;             % rotor inertia (kgm2)
%     Jg    	= 0.161e-7;         % gear inertia (kgm2)
%     Jl    	= 0.0387;           % load inertia (kgm2) (provisional value)
%     n 	    = 138;              % reduction
%     imax   = 0.72;              % maximum continuous current (A)
%     umax   = 6;                 % maximum input voltage (V)
%     wmax   = 6770*2*pi/60;      % maximum speed (rad/s)
%     
% %     wSun = 2;
%     b = Km*Kemf/R;
    
    
%% Simulation Parameters
%     N=15;      % time horizon (number of samples)
%     ts = 0.05;    % sampling time(s)
    
%% States and control
    DifferentialState w;        % Velocity 
    DifferentialState phi;      % Distance
    DifferentialState Econs;    % Dummy state
    DifferentialState Eprod;    % Dummy state
    
    Control u;                  % Control input
    TIME t;
    
%     AlgebraicState phiMotor;
%     AlgebraicState phiSun1;
    AlgebraicState current;
%     AlgebraicState phiSun;

% OnlineData winit phiinit Econsinit Eprodinit iter

%% Diferential Equation
    f = acado.DifferentialEquation();       % Set the differential equation object
    
    f.add(dot(w) == (current*Km-b*n*w)/n/(Jr+Jg+(Jl/(n*n))));
    f.add(dot(phi) == w);
    
    f.add(dot(Econs) == (u*current));
    
    f.add(dot(Eprod) == Pmax*ComputeEff(phi, ComputephiSun(t, iter, days, hdays, rango, func, do), flageff));%(-(sqrt((phiSun-phi).^2+0.0000001)))*Pmax);
    
    
    f.add(0==(u-Kemf*w*n)/R - current);
%     f.add(0==-phiSun+ComputephiSun(t, wSun));
%     f.add(0==-phiMotor+ComputephiSun(t, wSun)*n);
%     f.add(0==-phiSun1+wSun*t);
    

    
%     f.differentialList{3}.toString         % Print an equation to the screen 
    
    %% Optimal Control Problem
    ocp = acado.OCP((iter)*ts, (N+iter)*ts, N);          % Set up the Optimal Control Problem (OCP)
%     ocp = acado.OCP(19*ts, (N+19)*ts, N);
% ocp = acado.OCP(39*ts, (N+39)*ts, N);
                                            % Start at 0s, control in N
                                            % intervals upto n*ts s
    
                                            
    ocp.maximizeMayerTerm(wprod*Eprod-wcons*Econs);           % Minimize the consumed energy
% ocp.minimizeLSQ( [Econs; 1/Eprod] );
    
    ocp.subjectTo( f );                     % Optimize with respect to your differential equation
    ocp.subjectTo( 'AT_START', phi == phiinit ); 
    ocp.subjectTo( 'AT_START', w ==  winit ); 
    ocp.subjectTo( 'AT_START', Econs ==  Econsinit ); 
    ocp.subjectTo( 'AT_START', Eprod == Eprodinit );
    ocp.subjectTo( 'AT_START', current == iinit );
    
%         ocp.subjectTo( 'AT_START', phi == 2.6000 ); 
%     ocp.subjectTo( 'AT_START', w ==  -0.7828 ); 
%     ocp.subjectTo( 'AT_START', Econs ==  1.0553 ); 
%     ocp.subjectTo( 'AT_START', Eprod == -0.0336 );
    
%     ocp.subjectTo( 'AT_START', phi ==  -0.0668 ); 
%     ocp.subjectTo( 'AT_START', w ==  3.8766 ); 
%     ocp.subjectTo( 'AT_START', Econs ==  0.7215 ); 
%     ocp.subjectTo( 'AT_START', Eprod ==  -0.0332 );
    
%     ocp.subjectTo( 'AT_START', current ==  0.0 );
%     ocp.subjectTo( 'AT_START', phiSun ==  0 );


%     ocp.subjectTo( 'AT_END'  , phi == 100 ); 
%     ocp.subjectTo( 'AT_END'  , w ==  0.0 ); 
    ocp.subjectTo( -wmax <= w <= wmax );
    ocp.subjectTo( -umax <= u <= umax );
    ocp.subjectTo( -imax <= current <= imax );
%     ocp.subjectTo( 0<= u*current);
    
    %% Optimization Algorithm
    algo =acado.OptimizationAlgorithm(ocp); % Set up the optimization algorithm, link it to your OCP
%     algo.set( 'KKT_TOLERANCE', 1e-6 );      % Set a custom KKT tolerance
%     algo.set( 'VERBOSE', 0 );
    
    
END_ACADO;           % Always end with "END_ACADO".
                     % This will generate a file problemname_ACADO.m. 
                     % Run this file to get your results. You can
                     % run the file problemname_ACADO.m as many
                     % times as you want without having to compile again.

% Run the test
out = MyMotorNoInductance_RUN();


% figure;
% 
% subplot(2,3,1)
% plot(out.STATES(:,1), out.ALGEBRAICSTATES(:,2), 'r')
% title('Current');
% 
% subplot(2,3,2)
% plot(out.STATES(:,1), out.STATES(:,3), 'r')
% title('Angular position');
% hold;
% time = ts*(iter-1):ts:(N+iter-1)*ts;
% phisunplot = ComputephiSun(ts*(iter-1):ts:(N+iter-1)*ts, wSun);
% plot(time, phisunplot, 'b')
% legend('PhiPanel', 'PhiSun');
% hold off;
% 
% subplot(2,3,3)
% plot(out.STATES(:,1), out.CONTROLS(:,2).*(out.CONTROLS(:,2)-Kemf*out.STATES(:,2))/R, 'r')
% title('Power');
% 
% subplot(2,3,4)
% stairs(out.STATES(:,1), out.CONTROLS(:,2), 'r')
% title('Control');
% 
% subplot(2,3,5)
% plot(out.STATES(:,1), out.STATES(:,4), 'r')
% title('Consumed Energy');
% 
% subplot(2,3,6)
% plot(out.STATES(:,1), out.STATES(:,5), 'r')
% title('Generated Energy');

