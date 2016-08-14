clear
close all

disp('Loading...')
% Read data
data=heliread('../../validation/a109/data/trimdata.csv');
load('../../validation/a109/data/refdata/A109data.mat');
if (0)
dataFT.trim_power=heliread('../../validation/a109/data/refdata/A109_trim_power_FT.csv');
dataFT.trim_th0=heliread('../../validation/a109/data/refdata/A109_trim_th0_FT.csv');
dataFT.trim_ths=heliread('../../validation/a109/data/refdata/A109_trim_ths_FT.csv');
dataFT.trim_thc=heliread('../../validation/a109/data/refdata/A109_trim_thc_FT.csv');
dataFT.trim_thp=heliread('../../validation/a109/data/refdata/A109_trim_thp_FT.csv');
dataFT.trim_att=heliread('../../validation/a109/data/refdata/A109_trim_att_FT.csv');
dataNM.trim_power=heliread('../../validation/a109/data/refdata/A109_trim_power_NASAmodel.csv');
dataNM.trim_th0=heliread('../../validation/a109/data/refdata/A109_trim_th0_NASAmodel.csv');
dataNM.trim_ths=heliread('../../validation/a109/data/refdata/A109_trim_ths_NASAmodel.csv');
dataNM.trim_thc=heliread('../../validation/a109/data/refdata/A109_trim_thc_NASAmodel.csv');
dataNM.trim_thp=heliread('../../validation/a109/data/refdata/A109_trim_thp_NASAmodel.csv');
dataNM.trim_att=heliread('../../validation/a109/data/refdata/A109_trim_att_NASAmodel.csv');
end

hdata=heliread('../../validation/a109/data/refdata/trimdata_helilib.csv');

disp('Plotting....')
% Power required curve
figure(1)
plot(data.u,(data.Helicopter.powerreq)/1000,hdata.u,(hdata.Helicopter.powerreq)/1000,dataFT.trim_power.u,dataFT.trim_power.P/1000,'v',dataNM.trim_power.u,dataNM.trim_power.P/1000,'o-',dataEB.trim.u,dataEB.trim.P/1000+67,'x-');
title('Power required (with 67 kW bias)');
xlabel('u [m/s]');
ylabel('P [kW]');
legend('HeliSharp','HFEHeli','Flight test','Heffley','Backlund');
print '../../validation/a109/trimdata_power.png'

% Collective pitch
figure(2)
subplot(2,1,1);
plot(data.u,data.Helicopter.theta_0*180/pi,hdata.u,hdata.Helicopter.theta_0*180/pi,dataFT.trim_th0.u,dataFT.trim_th0.theta_0*180/pi,'v',dataNM.trim_th0.u,dataNM.trim_th0.theta_0*180/pi,'o-',dataEB.trim.u,dataEB.trim.theta_0*180/pi,'x-');
title('Collective pitch');
ylabel('Main Rotor \theta_0 [\circ]');
legend('HeliSharp','HFEHeli','Flight test','Heffley','Backlund');
subplot(2,1,2);
plot(data.u,data.Helicopter.theta_p*180/pi,hdata.u,hdata.Helicopter.theta_p*180/pi,dataFT.trim_thp.u,dataFT.trim_thp.theta_p*180/pi,'v',dataNM.trim_thp.u,dataNM.trim_thp.theta_p*180/pi,'o-',dataEB.trim.u,dataEB.trim.theta_p*180/pi,'x-');
xlabel('u [m/s]');
ylabel('Tail Rotor \theta_p [\circ]');
print '../../validation/a109/trimdata_collective.png'

% Cyclic pitch
figure(3)
subplot(2,1,1);
plot(data.u,data.Helicopter.theta_cos*180/pi,hdata.u,hdata.Helicopter.theta_cos*180/pi,dataFT.trim_thc.u,dataFT.trim_thc.theta_cos*180/pi,'v',dataNM.trim_thc.u,dataNM.trim_thc.theta_cos*180/pi,'o-',dataEB.trim.u,dataEB.trim.theta_cos*180/pi,'x-');
title('Cyclic pitch');
ylabel('Lateral \theta_{cos} [\circ]');
legend('HeliSharp','HFEHeli','Flight test','Heffley','Backlund');
subplot(2,1,2);
plot(data.u,data.Helicopter.theta_sin*180/pi,hdata.u,hdata.Helicopter.theta_sin*180/pi,dataFT.trim_ths.u,dataFT.trim_ths.theta_sin*180/pi,'v',dataNM.trim_ths.u,dataNM.trim_ths.theta_sin*180/pi,'o-',dataEB.trim.u,dataEB.trim.theta_sin*180/pi,'x-');
xlabel('u [m/s]');
ylabel('Longitudinal \theta_{sin} [\circ]');
print '../../validation/a109/trimdata_cyclic.png'

% Flapping
figure(4)
subplot(3,1,1);
plot(data.u,data.MainRotor.beta_cos*180/pi,hdata.u,hdata.MainRotor.beta_cos*180/pi,dataEB.trim.u,dataEB.trim.beta_cos*180/pi,'x-');
title('Flapping')
ylabel('Longitudinal \beta_{cos} [\circ]');
legend('HeliSharp','HFEHeli','Backlund');
subplot(3,1,2);
plot(data.u,data.MainRotor.beta_sin*180/pi,hdata.u,hdata.MainRotor.beta_sin*180/pi,dataEB.trim.u,dataEB.trim.beta_sin*180/pi,'x-');
ylabel('Lateral \beta_{sin} [\circ]');
subplot(3,1,3);
plot(data.u,data.MainRotor.beta_0*180/pi,hdata.u,hdata.MainRotor.beta_0*180/pi,dataEB.trim.u,dataEB.trim.beta_0*180/pi,'x-');
xlabel('u [m/s]');
ylabel('Coning \beta_0 [\circ]');
print '../../validation/a109/trimdata_flapping.png'

% Attitude
figure(5)
subplot(2,1,1);
plot(data.u,data.Helicopter.theta*180/pi,hdata.u,hdata.Helicopter.theta*180/pi,dataFT.trim_att.u,dataFT.trim_att.theta,'v',dataNM.trim_att.u,dataNM.trim_att.theta,'o-',dataEB.trim.u,dataEB.trim.theta*180/pi,'x-');
title('Attitude');
ylabel('Pitch \theta [\circ]');
legend('HeliSharp','HFEHeli','Flight test','Heffley','Backlund');
subplot(2,1,2);
plot(data.u,data.Helicopter.phi*180/pi,hdata.u,hdata.Helicopter.phi*180/pi,dataFT.trim_att.u,dataFT.trim_att.phi,'v',dataNM.trim_att.u,dataNM.trim_att.phi,'o-',dataEB.trim.u,dataEB.trim.phi*180/pi,'x-');
xlabel('u [m/s]');
ylabel('Roll \phi [\circ]');
print '../../validation/a109/trimdata_attitude.png'

if(0)
hgload('../../data/reffigs/power.fig');
plot(data.u,(data.MainRotor.CQ*1.2537e+09+data.TailRotor.CQ*3.9771e+07)/1000);
hgload('../../data/reffigs/collpitch.fig');
subplot(2,1,1);
plot(data.u,data.Helicopter.theta_0*180/pi)
subplot(2,1,2);
plot(data.u,data.Helicopter.theta_p*180/pi)
hgload('../../data/reffigs/cyclicpitch.fig');
subplot(2,1,1);
plot(data.u,data.Helicopter.theta_cos*180/pi)
subplot(2,1,2);
plot(data.u,data.Helicopter.theta_sin*180/pi)

hgload('../../data/reffigs/flapping.fig');
subplot(3,1,1);
plot(data.u,data.MainRotor.beta_cos*180/pi)
subplot(3,1,2);
plot(data.u,data.MainRotor.beta_sin*180/pi)
subplot(3,1,3);
plot(data.u,data.MainRotor.beta_0*180/pi)

hgload('../../data/reffigs/attitude.fig');
subplot(2,1,1);
plot(data.u,data.Helicopter.theta*180/pi)
subplot(2,1,2);
plot(data.u,data.Helicopter.phi*180/pi)
end

