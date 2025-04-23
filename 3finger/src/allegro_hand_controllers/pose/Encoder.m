clear all;
close all;
%% ENCODER CHECK

samplingtime = 0.002;
data= load('sensor.txt'); 
[r,c]=size(data);
t=1:r;
t=t*samplingtime;
dt= samplingtime:samplingtime:r*samplingtime;
for i = 1:3
    fingertip_sensor(:,i) = data(:,i);
    force(:,i) = data(:,i+3);
end


figure('Name', 'Fingertip_sensor');

[ax, h1, h2] = plotyy(...
    dt, fingertip_sensor, ...   
    dt, force);                 

set(h1, 'LineWidth', 2.5);
ylabel(ax(1), 'Fingertip Sensor [Pa]');
ylim(ax(1), [0 500]);


set(h2, 'LineWidth', 1);
ylabel(ax(2), 'Force [N]');
ylim(ax(2), [0 10]);


xlim(ax(1), [10 30]);
hold on;
xlim(ax(2), [10 30]);
xlabel('[sec]');

legend('Pressure(INDEX)', 'Pressure(MIDDLE)', 'Pressure(THUMB)','Force(INDEX)','Force(MIDDLE)','Force(THUMB)');

