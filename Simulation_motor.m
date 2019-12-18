% Simulator creation
% %% Parameters
%     Pmax 	= 290;               % max power (W) (provisional value)
%     L 	    = 0.0857e-3;   		% Motor terminal inductance (H)
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
%     b = Km*Kemf/R;
    
%% States and control
    DifferentialState w;        % Velocity 
    DifferentialState phi;      % Distance
    DifferentialState Econs;    % Dummy state
    DifferentialState Eprod;    % Dummy state
    DifferentialState cur;
    
    Control u;                  % Control input

    AlgebraicState current;
    
    OnlineData eff
% current = is((u-Kemf*w*n)/R);

%% Differential Equations
    f = acado.DifferentialEquation();       % Set the differential equation object
    
    f.add(dot(w) == (current*Km-b*n*w)/n/(Jr+Jg+(Jl/(n*n))));
    f.add(dot(phi) == w);
    f.add(dot(Econs) == (u*current));
    f.add(dot(Eprod) == Pmax*eff);
    f.add(0==(u-Kemf*w*n)/R - current);
    
    
    fsim = acado.DifferentialEquation();
    fsim.add(dot(w) == (cur*Km-b*n*w)/n/(Jr+Jg+(Jl/(n*n))));
    fsim.add(dot(phi) == w);
    fsim.add(dot(Econs) == (u*cur));
    fsim.add(dot(Eprod) == Pmax*eff);
    fsim.add(dot(cur)==(u-Kemf*n*w-R*cur)/L);
    
%% SIMexport
    acadoSet('problemname', 'SIMMotor');

    sim = acado.SIMexport( tsim );
    sim.setModel(fsim);
    sim.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL4'   );
    sim.set( 'NUM_INTEGRATOR_STEPS',        2               );

    
    sim.exportCode('export_SIM');
    cd export_SIM
    make_acado_integrator('../integrate_vdp')
    cd ..

    
    