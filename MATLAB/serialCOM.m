fclose(instrfindall)
clc;clear
s=serialport('/dev/ttyACM0',115200);
values = [];
while true
    if s.NumBytesAvailable > 0
        data = readline(s);
        fprintf(data)
        values = [values;data];
    end
end
%% Solo si se desea guardar datos, usar lo siguiente
%save("values_at_60","values");
