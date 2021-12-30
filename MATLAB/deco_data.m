fclose(instrfindall)
clc;clear
load("values_at_50.mat");
%%
fprintf("\nRun Codigo (@50%%)")
temp=char(values(2:end,:));
for i=1:length(temp)/2
    if ('0'~=temp(2*i,5))%&&('1'~=temp(2*i,5))
        fprintf("\nEl valor del indice es %d",i);
        fprintf(temp(2*i,:));
    end
end
fprintf("\n");
%%
%Se tiene que el tiempo entre trama y trama es de 256us aprox
Velocidad=60000/((263-24)*0.256)
promVel=[Velocidad];
Velocidad=60000/((498-263)*0.256)
promVel=[promVel Velocidad];
Velocidad=60000/((728-498)*0.256)
promVel=[promVel Velocidad];
Velocidad=60000/((955-728)*0.256)
promVel=[promVel Velocidad];
Velocidad=60000/((1178-955)*0.256)
promVel=[promVel Velocidad];
Velocidad=60000/((1398-1178)*0.256)
promVel=[promVel Velocidad];
promVel=mean(promVel)

