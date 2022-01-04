clear;clc;close all
load("rplidar_cartesian_data_1")
i=2;
data=[];
while (i<length(values))
    data=[data;str2num(values(i,:))];
    i=i+1;
end
%% Ploteamos la data
%No olvidar colocar el signo al valor de data y añadir el offset angular
%según corresponda. En este caso el offset es de 90/2 y el signo es
%positivo. La vista es similar al archivo de valid_data_1.
rot_angle=-11;
rot_matrix=[cosd(rot_angle) -sind(rot_angle);
            sind(rot_angle)  cosd(rot_angle)];
data=(rot_matrix*data')';
scatter(data(:,1),data(:,2));