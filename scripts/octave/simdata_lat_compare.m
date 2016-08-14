graphics_toolkit('fltk')

close all
simdataplot('../../validation/a109/data/simdata_lat.csv');
hdata=heliread('../../validation/a109/data/refdata/simdata_lat_helilib.csv')
dataFT=heliread('../../validation/a109/data/refdata/A109_sim_lat_right_pulse.csv');
tmax=6;

% Overlay data from flight test and set axis like Backlund report figures

figure(1)
hold all
plot(hdata.t,[hdata.Helicopter.theta_0 hdata.Helicopter.theta_sin hdata.Helicopter.theta_cos hdata.Helicopter.theta_p]*180/pi);

figure(2)
hold all
plot(hdata.t,[hdata.MainRotor.beta_0 hdata.MainRotor.beta_sin hdata.MainRotor.beta_cos]*180/pi);

figure(3)

subplot(3,3,1)
hold all
plot(hdata.t,hdata.Helicopter.u);
axis([0 tmax -2 1]);

subplot(3,3,2)
title('Lateral input response')
hold all
plot(hdata.t,hdata.Helicopter.v);
axis([0 tmax -5 10]);

subplot(3,3,3)
hold all
plot(hdata.t,hdata.Helicopter.w);
axis([0 tmax -1 1]);

subplot(3,3,4)
hold all
plot(hdata.t,hdata.Helicopter.p*180/pi);
plot(dataFT.t,dataFT.p,'r.-');
axis([0 tmax -20 40]);

subplot(3,3,5)
hold all
plot(hdata.t,hdata.Helicopter.q*180/pi);
plot(dataFT.t,dataFT.q,'r.-');
axis([0 tmax -10 20]);

subplot(3,3,6)
hold all
plot(hdata.t,hdata.Helicopter.r*180/pi);
plot(dataFT.t,dataFT.r,'r.-');
axis([0 tmax -5 15]);

subplot(3,3,7)
hold all
plot(hdata.t,hdata.Helicopter.phi*180/pi);
plot(dataFT.t,dataFT.phi,'r.-');
axis([0 tmax -20 40]);

subplot(3,3,8)
hold all
plot(hdata.t,hdata.Helicopter.theta*180/pi);
axis([0 tmax 0 10]);

subplot(3,3,9)
hold all
plot(hdata.t,hdata.Helicopter.psi*180/pi);
axis([0 tmax -10 20]);

figure(4)
subplot(2,1,1)
hold all
plot(hdata.t,hdata.Engine.Omega*9.5492966)
subplot(2,1,2)
hold all
plot(hdata.t,hdata.Engine.Qeng,hdata.t,hdata.Engine.Qload,'--')


figure(3)
print '../../validation/a109/simdata_lat_velocities.png'
