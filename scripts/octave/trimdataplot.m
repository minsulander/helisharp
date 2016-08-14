%TRIMDATAPLOT Plot trim data from HeliLib
%   [data,header] = TRIMDATAPLOT(file)
%   see also HELIREAD for file formats and such

function [data,header] = trimdataplot(file)

disp('Loading...')
[data,header]=heliread(file);

disp('Plotting...')

% Power required curve
figure(1)
plot(data.u,(data.Helicopter.powerreq)/1000);
title('Power required');
xlabel('u [m/s]');
ylabel('P [kW]');

% Collective pitch
figure(2)
subplot(2,1,1);
plot(data.u,data.Helicopter.theta_0*180/pi);
title('Collective pitch');
ylabel('Main Rotor \theta_0 [\circ]');
subplot(2,1,2);
plot(data.u,data.Helicopter.theta_p*180/pi);
xlabel('u [m/s]');
ylabel('Tail Rotor \theta_p [\circ]');

% Cyclic pitch
figure(3)
subplot(2,1,1);
plot(data.u,data.Helicopter.theta_cos*180/pi);
title('Cyclic pitch');
ylabel('Lateral \theta_{cos} [\circ]');
subplot(2,1,2);
plot(data.u,data.Helicopter.theta_sin*180/pi);
xlabel('u [m/s]');
ylabel('Longitudinal \theta_{sin} [\circ]');

% Flapping
figure(4)
subplot(3,1,1);
plot(data.u,data.MainRotor.beta_cos*180/pi);
title('Flapping')
ylabel('Longitudinal \beta_{cos} [\circ]');
subplot(3,1,2);
plot(data.u,data.MainRotor.beta_sin*180/pi);
ylabel('Lateral \beta_{sin} [\circ]');
subplot(3,1,3);
plot(data.u,data.MainRotor.beta_0*180/pi);
xlabel('u [m/s]');
ylabel('Coning \beta_0 [\circ]');

% Attitude
figure(5)
subplot(2,1,1);
plot(data.u,data.Helicopter.theta*180/pi);
title('Attitude');
ylabel('Pitch \theta [\circ]');
subplot(2,1,2);
plot(data.u,data.Helicopter.phi*180/pi);
xlabel('u [m/s]');
ylabel('Roll \phi [\circ]');

