%SIMDATAPLOT Plot simulation data from HeliLib
%   [data,header] = SIMDATAPLOT(file)
%   see also HELIREAD for file formats and such

function [data,header] = simdataplot(file)

disp('Loading...')
[data,header]=heliread(file);

disp('Plotting...')

figure(1)
plot(data.t,[data.Helicopter.theta_0 data.Helicopter.theta_sin data.Helicopter.theta_cos data.Helicopter.theta_p]*180/pi);
legend('t0','ts','tc','tp');
title(strcat(['Control angles - ' header]))
xlabel('t [s]')
ylabel('[deg]')

figure(2)
plot(data.t,[data.MainRotor.beta_0 data.MainRotor.beta_sin data.MainRotor.beta_cos]*180/pi);
legend('b0','bs','bc');
title(strcat(['Flapping angles - ' header]))
xlabel('t [s]')
ylabel('[deg]')

figure(3)

subplot(3,3,1)
plot(data.t,data.Helicopter.u);
ylabel('u [m/s]')

subplot(3,3,2)
plot(data.t,data.Helicopter.v);
ylabel('v [m/s]')

subplot(3,3,3)
plot(data.t,data.Helicopter.w);
ylabel('w [m/s]')

subplot(3,3,4)
plot(data.t,data.Helicopter.p*180/pi);
ylabel('p [deg/s]')

subplot(3,3,5)
plot(data.t,data.Helicopter.q*180/pi);
ylabel('q [deg/s]')

subplot(3,3,6)
plot(data.t,data.Helicopter.r*180/pi);
ylabel('r [deg/s]')

subplot(3,3,7)
plot(data.t,data.Helicopter.phi*180/pi);
ylabel('phi [deg]')

subplot(3,3,8)
plot(data.t,data.Helicopter.theta*180/pi);
ylabel('theta [deg]')

subplot(3,3,9)
plot(data.t,data.Helicopter.psi*180/pi);
ylabel('psi [deg]')

figure(4)
subplot(2,1,1)
title(strcat(['Engine - ' header]))
plot(data.t,data.Engine.Omega*9.5492966)
ylabel('\Omega [R/min]')
subplot(2,1,2)
plot(data.t,data.Engine.Qeng,data.t,data.Engine.Qload,'--')
ylabel('Q [Nm]')
xlabel('t [s]')
legend('Output','Load')

%figure(5)
%plot(data.t,data.MainRotor.lambda_i0,data.t,data.TailRotor.lambda_i0)
%ylabel('\lambda_{i0}')
%xlabel('t [s]')
%title('Inflow')
%legend('Main Rotor','Tail Rotor')
