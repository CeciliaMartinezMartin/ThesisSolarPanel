% Simulator creation
    
%% States and control
DifferentialState w;        % Velocity 
DifferentialState phi;      % Distance
DifferentialState Econs;    % Dummy state
DifferentialState Eprod;    % Dummy state
DifferentialState cur;

Control u;                  % Control input

OnlineData eff;

%% Differential Equations
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
