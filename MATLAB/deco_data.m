clc;clear;close all
%load("values_at_60_v3.mat");        
%load("rplidar_data_1.mat");        
load("valid_data_2.mat");
%% Volvemos a obtener la data cruda para manipularla
raw_data=uint16(zeros(length(values)/2-1,5));
for i=1:length(values)/2-1
    temp=strip(values(i*2));
    raw_data(i,:)=hex2dec(split(temp,"-")');
end
% %% Creamos el plot con los datos medidos
% n_points=length(raw_data);
% for i=1:n_points
%     angle=single(bitshift(raw_data(i,3),7))+single(bitshift(raw_data(i,2),-1));
%     angle=angle/64;
%     distance=single(bitshift(raw_data(i,5),8))+single(raw_data(i,4));
%     distance=distance/4/1000;
%     data(i,:)=[angle,distance];
% end
% polarscatter(pi/2+data(:,1)*pi/180,data(:,2))
%% Dividimos la data en los 4 diferentes escaneos
n_points=length(raw_data);
index=0;
count=0;
for i=1:n_points
    if (bitand(raw_data(i,1),0x0003))==1
        index=index+1;
        offset=i-1;
        count
        count=0;
    end
    angle=single(bitshift(raw_data(i,3),7))+single(bitshift(raw_data(i,2),-1));
    angle=angle/64;
    distance=single(bitshift(raw_data(i,5),8))+single(raw_data(i,4));
    distance=distance/4/1000;
    data{1,index}(i-offset,:)=[angle,distance];
    if distance==0 
        count=count+1;
    end
end
figure(1)
for i=1:length(data)
    temp=data{i};
    polarscatter(pi/2+temp(:,1)*pi/180,temp(:,2))
    hold on
end
title('All scans')
figure
for i=1:length(data)
    temp=data{i};
    subplot(2,3,i)
    polarscatter(pi/2+temp(:,1)*pi/180,temp(:,2))
    
    title(sprintf('Scan #%d',i));
    hold on
end