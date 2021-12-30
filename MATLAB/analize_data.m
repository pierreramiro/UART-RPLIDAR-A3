%fclose(instrfindall)
clc;clear
%file="longvalues_at_60.mat";
%file="values_at_60.mat";
file="values_at_60_v3.mat";
load(file);
%%
%file="values_at_60_v2.mat";
fprintf("\nRun Codigo "+file)
%transformamos la data en caracteres
temp=char(values(2:end,:));
%Analizamos la data para obtener las líneas con S: 1
id_array=[];
for i=1:length(temp)/2
    %Condicional para "S: 1"
    %if ('0'~=temp(2*i,5))%valu&&('1'~=temp(2*i,5))
    if ('1'==temp(2*i,5))
        fprintf("\nEl valor del indice es %d",i);
        fprintf(temp(2*i,:));
        id_array=[id_array,i];
    end
end
for i=1:length(id_array)-1
    id_diff(i)=id_array(i+1)-id_array(i);
end
fprintf("\n\nRun Codigo "+file)
fprintf("\n");
id_diff
sum_id_diff=sum(id_diff)
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

