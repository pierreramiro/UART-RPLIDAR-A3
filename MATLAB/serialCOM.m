%fclose(instrfindall)
clc;clear
s=serialport('/dev/ttyACM0',115200);%For Linux
%s=serialport('COM7',115200);%For Windows
values = [];
while true
    if s.NumBytesAvailable > 0    
        data = readline(s);
        fprintf(data)
        values = [values;data]; 
    end
end
%%
save("rplidar_cartesian_data_4","values");
%save("longvalues_at_60_long","values");


%%
%Coordenadas UTM
%Altura/cota del puntoo A
%En caso no hallar la cota, pedir al topografo un punto BASE,
%Al georeferencias no va a tener una alta presición, pero tampoco una baja,
%estan en la MEDIA
%formato para coordenadas->LAS. Normalmente de hace un procesamiento de
%cartesianas a UTM

