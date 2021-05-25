%% Lets plot our partial results

figure;
plot(xstatesx(1,:))
ylabel('x(t)'),title('State')

figure;
plot(xstatesx(2,:))
ylabel('y(t)'),title('State')

figure;
plot(xstatesx(3,:))
ylabel('z(t)'),title('State')

figure;
plot(xstatesx(4,:))
ylabel('psi(t)'),title('State')

figure;
plot(xstatesx(5,:))
ylabel('theta(t)'),title('State')

figure;
plot(xstatesx(6,:))
ylabel('phi(t)'),title('State')

figure;
plot(rad2deg(xstatesx(7,:)))
ylabel('beta1(t) [deg]'),title('State')

figure;
plot(rad2deg(xstatesx(8,:)))
ylabel('beta2(t) [deg]'),title('State')

figure;
plot(xstatesx(9,:))
ylabel('xdot(t)'),title('State')

figure;
plot(xstatesx(10,:))
ylabel('ydot(t)'),title('State')

figure;
plot(xstatesx(11,:))
ylabel('zdot(t)'),title('State')

figure;
plot(xstatesx(12,:))
ylabel('psidot(t)'),title('State')

figure;
plot(xstatesx(13,:))
ylabel('thetadot(t)'),title('State')

figure;
plot(xstatesx(14,:))
ylabel('phidot(t)'),title('State')

figure;
plot(xstatesx(15,:))
ylabel('beta1dot(t)'),title('State')

figure;
plot(xstatesx(16,:))
ylabel('beta2dot(t)'),title('State')


%% Plot xbarrax
figure;
plot(xbarrax(1,:))
ylabel('xbarra1(t)'),title('State')

figure;
plot(xbarrax(2,:))
ylabel('xbarra2(t)'),title('State')

figure;
plot(xbarrax(3,:))
ylabel('xbarra3(t)'),title('State')

figure;
plot(xbarrax(4,:))
ylabel('xbarra4(t)'),title('State')

figure;
plot(xbarrax(5,:))
ylabel('xbarra5(t)'),title('State')

figure;
plot(xbarrax(6,:))
ylabel('xbarra6(t)'),title('State')

figure;
plot(rad2deg(xbarrax(7,:)))
ylabel('beta1barra(t) [deg]'),title('State')

figure;
plot(rad2deg(xbarrax(8,:)))
ylabel('beta2barra(t) [deg]'),title('State')

figure;
plot(xbarrax(9,:))
ylabel('xbarra9 velocitat M1(t)'),title('State')

figure;
plot(xbarrax(10,:))
ylabel('xbarra10 velocitat M2(t)'),title('State')

figure;
plot(xbarrax(11,:))
ylabel('xbarra11 velocitat M3(t)'),title('State')

figure;
plot(xbarrax(12,:))
ylabel('xbarra12 velocitat M4(t)'),title('State')

figure;
plot(xbarrax(13,:))
ylabel('xbarra13 velocitat M5(t)'),title('State')

figure;
plot(xbarrax(14,:))
ylabel('xbarra14 velocitat M6(t)'),title('State')

figure;
plot(xbarrax(15,:))
ylabel('beta1dotbarra(t)'),title('State')

figure;
plot(xbarrax(16,:))
ylabel('beta2dotbarra(t)'),title('State')
