%fclose(instrfindall)
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
%%
save("values_at_60_v3","values");
%save("longvalues_at_60_long","values");
