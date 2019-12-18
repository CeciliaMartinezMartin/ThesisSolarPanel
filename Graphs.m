%Plot graphs and save in a correct format

len = size(uhist,2);
do.add =0;
do.subs = 0;

upper = 180;
lower = -180;
fig = figure()
clf;

% Plot for current
subplot(4,3,1)
plot(0:ts:(size(ihist, 2)-1)*tsim, ihist(1:Nsim:size(whist,2)), 'b')
title('Current (A)');
xlabel ('Time (s)');
xlim ([0, len*ts-ts]);

% Plot for phi
subplot(2,1,2)
while (~isempty(phihist(phihist<lower/360*2*pi)))
        phihist(phihist<lower/360*2*pi) = phihist(phihist<lower/360*2*pi) +360/360*2*pi;
end
while (~isempty(phihist(phihist>=upper/360*2*pi)))
        phihist(phihist>=upper/360*2*pi) = phihist(phihist>upper/360*2*pi) -360/360*2*pi;
end
plot(0:ts:(size(phihist, 2)-1)*tsim, phihist(1:Nsim:size(whist,2))*360/2/pi, 'b')
title('Angular position (degrees)');
hold;
plot(data.t, (ComputephiSunAbsolute(0:20:Tcontrol+N*ts, days, hdays, rango)-data.phi)*360/2/pi, 'o','MarkerEdgeColor', 'r');
legend('PhiPanel','PhiSun', 'Location', 'southeast');
hold off;
xlabel ('Time (s)');
xlim ([0, len*ts-ts]);
ylim ([-80, 10])

% Plot for power
subplot(4,3,2)
plot(0:ts:(size(phist, 2)-1)*tsim, phist(1:Nsim:size(whist,2)), 'b')
title('Power (W)');
xlabel ('Time (s)');
xlim ([0, len*ts-ts]);

% Plot for control signal
subplot(2,1,1)
stairs(0:ts:(size(uhist, 2)-1)*ts, uhist, 'b')
title('Control input (V)');
xlim ([0, len*ts-ts]);
xlabel ('Time (s)');

% Plot for consumed energy
subplot(4,3,[3 6])
plot(0:ts:(size(echist, 2)-1)*tsim, echist(1:Nsim:size(whist,2)), 'b')
title('Consumed Energy (J)');
xlim ([72, len*ts-ts]);
xlabel ('Time (s)');

% Plot for generated energy
subplot(4,3,[9 12])
plot(0:ts:(size(echist, 2)-1)*tsim, ephist(1:Nsim:size(whist,2)), 'b')
title('Generated Energy (J)');
xlim ([0, len*ts-ts]);
xlabel ('Time (s)');

% Plot for efficiency
subplot(4,3,5)
stairs(0:ts:(size(effhist, 2)-1)*tsim, effhist(1:Nsim:size(whist,2))*100, 'b')
title('Efficiency (%)');
xlim ([72, len*ts-ts]);
xlabel ('Time (s)');

% Plot for angular speed
subplot(4, 3, 7:8)
plot(0:ts:(size(whist, 2)-1)*tsim, whist(1:Nsim:size(whist,2)), 'b')
title('Speed (rad/s)');
xlim ([0, len*ts-ts]);
xlabel ('Time (s)');

cd plots
name = strcat('T_',num2str(iter*ts));
saveas(fig, strcat(name,'.eps'), 'epsc');
saveas(fig, strcat(name,'.fig'));
cd ..



