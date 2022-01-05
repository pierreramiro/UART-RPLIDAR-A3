%fclose(instrfindall)
clc;clear
%s=serialport('/dev/ttyACM0',115200);%For Linux
s=serialport('COM7',115200);%For Windows
values = [];
while true
    if s.NumBytesAvailable > 0
        data = readline(s);
        fprintf(data)
        values = [values;data];
    end
end
%%
save("rplidar_cartesian_data_1","values");
%save("longvalues_at_60_long","values");
