clear;clc;close all
load("rplidar_data_1.mat")
i=3;
data=[];
while (i<length(values))
    data=[data;str2num(values(i,:))];
    i=i+2;
end
%% Ploteamos la data
%No olvidar colocar el signo al valor de data y añadir el offset angular
%según corresponda. En este caso el offset es de 90/2 y el signo es
%positivo. La vista es similar al archivo de valid_data_1.
polarscatter(pi/2+data(:,1)*pi/180,data(:,2))