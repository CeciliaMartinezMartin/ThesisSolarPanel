% Optimal Control Problem solution
BEGIN_ACADO;                                
    
acadoSet('problemname', 'MyMotorNoInductance');     %Set the name of the problem

%% States and control
DifferentialState w;        % Velocity 
DifferentialState phi;      % Distance
DifferentialState Econs;    % Consumed energy
DifferentialState Eprod;    % Generated energy

Control u;                  % Control input
TIME t;

AlgebraicState current;

%% Diferential Equation
f = acado.DifferentialEquation();       % Set the differential equation object

f.add(dot(w) == (current*Km-b*n*w)/n/(Jr+Jg+(Jl/(n*n))));
f.add(dot(phi) == w);
f.add(dot(Econs) == (u*current));
f.add(dot(Eprod) == Pmax*ComputeEff(phi, ComputephiSun(t, days, hdays, rango, func, do), flageff));%(-(sqrt((phiSun-phi).^2+0.0000001)))*Pmax);

f.add(0==(u-Kemf*w*n)/R - current); 

%% Optimal Control Problem
ocp = acado.OCP((iter)*ts, (N+iter)*ts, N);          % Set up the Optimal Control Problem (OCP)
ocp.maximizeMayerTerm(wprod*Eprod-wcons*Econs);           % Objective function
    
ocp.subjectTo( f );                     % Optimize with respect to your differential equation
ocp.subjectTo( 'AT_START', phi == phiinit ); 
ocp.subjectTo( 'AT_START', w ==  winit ); 
ocp.subjectTo( 'AT_START', Econs ==  Econsinit ); 
ocp.subjectTo( 'AT_START', Eprod == Eprodinit );
ocp.subjectTo( 'AT_START', current == iinit );
    
ocp.subjectTo( -wmax <= w <= wmax );
ocp.subjectTo( -umax <= u <= umax );
ocp.subjectTo( -imax <= current <= imax );

%% Optimization Algorithm
algo =acado.OptimizationAlgorithm(ocp); % Set up the optimization algorithm, link it to your OCP

END_ACADO;           % Always end with "END_ACADO".
                     % This will generate a file problemname_ACADO.m. 
                     % Run this file to get your results. You can
                     % run the file problemname_ACADO.m as many
                     % times as you want without having to compile again.

% Run the test
out = MyMotorNoInductance_RUN();
