clear;clc;close all
data=readmatrix("final.csv");
%% Ploteamos la data
%No olvidar colocar el signo al valor de data y añadir el offset angular
%según corresponda. En este caso el offset es de 90/2 y el signo es
%positivo. La vista es similar al archivo de valid_data_1.
rot_angle=0;
rot_matrix=[cosd(rot_angle) -sind(rot_angle);
            sind(rot_angle)  cosd(rot_angle)];
%La siguiente operación se hace solo para tener un ajuste angular en el
%plot. Simplemente para visualizar bien la data
data=(rot_matrix*data')';
%En caso el RPLIDAR está volteado, basta con cambiar de signo a la
%componente 'x' y realizar un ajuste de angulo si es necesario 

scatter(data(:,1),data(:,2));
%% Analisis de los indices
%init_index=1;
%end_index=200;
%scatter(data(init_index:end_index,1),data(init_index:end_index,2));