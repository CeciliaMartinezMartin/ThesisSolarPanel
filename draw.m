
subplot(4,3,1)
plot(timecontrol(2:end), icontrol(2:end), 'r')
hold on
plot(0:tsim:(size(ihist, 2)-1)*tsim, ihist, 'b')
hold off
title('Current');
xlim ([0, Tcontrol + N*ts]);

subplot(4,3,10:11)
plot(timecontrol(2:end), phicontrol(2:end), 'r', 0:tsim:(size(phihist, 2)-1)*tsim, phihist, 'b')
title('Angular position');
hold;
time = 0:ts:Tcontrol+N*ts;
phisunplot = ComputephiSun(0:ts:Tcontrol+N*ts, iter, days, hdays, rango, func, do);
plot(time, phisunplot, 'y', ts*(1+iter:iter+N),phisunplot((2+iter):iter+N+1),'g')
plot(data.t, ComputephiSunAbsolute(0:20:Tcontrol+N*ts, days, hdays, rango)-data.phi, 'k');
legend('PhiPanel','Implemented', 'PhiSunTemp', 'PhiSunHorizon', 'PhiSun', 'Location', 'southwest');
hold off;
xlim ([0, Tcontrol + N*ts]);


subplot(4,3,2)
plot(timecontrol(2:end), ucontrol(2:end).*icontrol(2:end), 'r', 0:tsim:(size(phist, 2)-1)*tsim, phist, 'b')
title('Power');
xlim ([0, Tcontrol + N*ts]);


subplot(4,3,4)
stairs(timecontrol(1:end-1), ucontrol(2:end), 'r')
hold on;
stairs(0:ts:(size(uhist, 2)-1)*ts, uhist, 'b')
title('Control');
hold off;
xlim ([0, Tcontrol + N*ts]);


subplot(4,3,[3 6])
plot(timecontrol(2:end), eccontrol(2:end), 'r', 0:tsim:(size(echist, 2)-1)*tsim, echist, 'b')
title('Consumed Energy');
xlim ([0, Tcontrol + N*ts]);


subplot(4,3,[9 12])
plot(timecontrol(2:end), epcontrol(2:end), 'r', 0:tsim:(size(ephist, 2)-1)*tsim, ephist, 'b')
title('Generated Energy');
xlim ([0, Tcontrol + N*ts]);


subplot(4,3,5)
stairs(timecontrol(2:end), ComputeEff(phicontrol(2:end)',ComputephiSun(ts*(iter+1):ts:N*ts+ts*(iter),iter, days, hdays, rango, func, do), flageff), 'r')
hold on
stairs(0:tsim:(size(effhist, 2)-1)*tsim, effhist, 'b')
hold off
title('Efficiency');
xlim ([0, Tcontrol + N*ts]);

subplot(4, 3, 7:8)
plot(timecontrol(2:end), wcontrol(2:end), 'r', 0:tsim:(size(whist, 2)-1)*tsim, whist, 'b')
title('Speed');
xlim ([0, Tcontrol + N*ts]);