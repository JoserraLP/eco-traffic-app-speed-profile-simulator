clear
clc

format longG

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%          Datos del vehículo         %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

m_v=1400;         % Masa del vehículo en vacío en kg
m_carga=300;      % Masa de las carga del vehículo en kg
m_t=m_v+m_carga;  % Masa total del vehículo
  
gamma=1.05;       % Factor de mayoración de masas rotativas

%Resistencias al avance R=A+B·v(t)+C·v^2·(t)+m_t·g·sin?
fr=0.015;         % Coeficiente de resistencias al avance 
A= m_t*fr*9.81;   % Parámetro en N
B= 0;             % Parámentro en N/(m/s)  

% Efecto aerodinámico
Af=2;             % Área frontal en metros
ro=1.225;         % Densidad del aire en kg/m^3
Cx=0.3;           % Coeficiente aerodinámico

C=0.5*ro*Af*Cx;   % Valor del parámetro C, función de la v^2. Expresado en N/(m/s)^2

% Si se disponen de otros valores más precisos, se pueden introducir
% directamente para el cálculo de las resistencias pasivas, Cuidado con las
% unidades, ya que habitualmente suelen estar expresados para una velocidad
% en km/h y no en m/s


% Definición de los criterios de conducción. Se establecen tres franjas de conducción 

v1_km_h=50;      % Primer límite de velocidad en km/h
v2_km_h=100;     % Segundo límite de velocidad en km/h

ax_trac1=2;      % Aceleración longitudinal de tracción máxima en m/s^2 con velocidad menor que v1_km_h
ax_trac2=1;      % Aceleración longitudinal de tracción máxima en m/s^2 con velocidad entre v1_km_h y v2_km_h
ax_trac3=0.5;    % Aceleración longitudinal de tracción máxima en m/s^2 con velocidad mayor que v2_km_h

ax_brake=-2;     % Aceleración longitudinal de frenada en m/s^2 

delta_t=0.1;     % Incremento de tiempo entre pasos

Pmax_kW=100;        % Potencia máxima en kW del motor

% Valor de porcentaje de la pontencia máxima para el cálculo del
% rendimiento según FASTSIM
X_porcentaje=[0 0.5 1.5 4 6 10 14 20 40 60 80 100]; 

% Valor del rendimiento según FASTSIM
Rendimiento_motor=[10 14 20 26 32 39 41 42 41 38 36 34];

kWh_por_l_diesel=10;                 % Conversión de litros gasóleo a kWh 
l_diesel_por_kWh=1/kWh_por_l_diesel; % Inverso del anterior para obtener litros de gasóleo por kWh

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% Definición del trayecto %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% En esta parte se introducen los datos que configuran el recorrido a
% analizar. El trayecto se define por una sucesión de tramos. Cada tramo
% está definido por las siguientes matrices:
%   pk_inicio:     Variable que indica el punto de inicio de cada tramo, 
%                  medida en metros desde el origen.
%   v_limite_km_h: Velocidad límite de cada tramo, expresada en km/h
%   rampa:         Rampa del tramo, expresada en grados sexagesimanles 





meta=5260;                        % Longitud total del recorrido

% La definición del recorrido se caracteriza por la distancia al origen del
% inicio de cada tramo, medida en metros (m)

pk_inicio=[0
127.467
180.666
210.155
277.484
335.355
385.251
418.889
451.944
546.883
565.302
608.156
650.325
671.923
700.137
911.484
969.517
1023.979
1195.154
1258.969
1493.249
1545.19
1639.779
1660.423
1692.078
1733.973
1777.712
1807.215
1859.316
1932.884
1976.292
2045.448
2075.643
2126.936
2170.978
2235.555
2256.985
2301.196
2317.233
2370.216
2424.61
2486.11
2571.955
2609.579
2637.789
2682.104
2702.238
2747.368
2785.704
2801.271
2829.93
2866.397
2882.906
2912.449
2969.646
3000.088
3048.231
3073.721
3112.699
3143.962
3166.457
3173.294
3193.212
3227.409
3252.349
3297.097
3349.735
3487.385
3537.84
3570.729
3621.916
3635.727
3682.136
3751.879
3780.272
3784.487
3804.28
3851.058
3930.629
3959.784
3974.473
4006.034
4024.854
4047.718
4085.93
4119.758
4146.509
4167.222
4191.544
4349.483
4432.747
4449.532
4498.898
4629.658
4656.788
4740.748
4784.98
4841.944
4880.016
4918.913
4951.792
4995.102
5023.51
5095.255
5122.771
5208.882
5271.667
];   % Vector de inicio de tramo


% Velocidades límite en cada tramo, obtenido de la máxima del tramo,
% modificada por el estado del tráfico. Está expresada en km/h

v_limite_km_h=[50
51.96152423
42.42640687
60
35.4964787
30
50
26.83281573
50
30
50
21.21320344
50
23.23790008
120
60
32.86335345
50
51.96152423
50
60
50
26.83281573
21.21320344
32.86335345
50
37.94733192
37.94733192
73.48469228
35.4964787
50
21.21320344
30
25.0998008
50
30
50
18.97366596
50
30
51.96152423
51.96152423
26.83281573
18.97366596
35.4964787
37.94733192
18.97366596
50
23.23790008
50
21.21320344
50
18.97366596
51.96152423
23.23790008
51.96152423
21.21320344
50
26.83281573
50
13.41640786
50
32.86335345
26.83281573
50
35.4964787
79.37253933
46.47580015
50
32.86335345
50
25.0998008
50
26.83281573
18.97366596
50
26.83281573
35.4964787
37.94733192
50
42.42640687
50
18.97366596
50
18.97366596
32.86335345
50
18.97366596
50
30
50
25.0998008
46.47580015
30
60
30
50
32.86335345
32.86335345
42.42640687
51.96152423
42.42640687
50
14.69693846
84.85281374
103.9230485
84.85281374
16.43167673
50
42.42640687
14.69693846
50
60
60
50
25.0998008
50
42.42640687
50
25.0998008
50
32.86335345
50
26.83281573
32.86335345
42.42640687
50
18
50
51.96152423
67.08203932
23.23790008
42.42640687
25.0998008
51.96152423
37.94733192
50
37.94733192
50
23.23790008
50
16.43167673
46.47580015
73.48469228
30
50
46.47580015
51.96152423
37.94733192
50
26.83281573
30
50
21.21320344
42.42640687
50
15.87450787
42.42640687
60
30
50
26.83281573
50
28.46049894
50
37.94733192
50
60
60
60
73.48469228
50
26.83281573
30
50
42.42640687
50
25.0998008
50
50
];                       % Velocidades limite de tramo




% Rampa de cada tramo expresada en %. Se considera que la
% rampa(%)=100*tan(inclinación)=100*seno(inclinación)

rampa=[1.976319949
6.723120265
6.723278172
6.722783644
6.723272019
6.722995831
6.722192312
6.457861141
4.450571578
2.572137195
1.555507204
0.706804271
0.685
0.685
0.685
0.685
0.685
0.818247378
2.933258325
6.218465447
6.673074739
6.673942795
6.675
6.673092245
6.674054541
6.672975262
6.674612243
6.675
6.673640713
6.672696277
6.673553994
6.675
6.673050416
6.675
6.673451461
6.675
6.672738119
6.675
6.674035917
6.674100636
6.673373984
6.67383511
6.673727009
6.67315296
6.674762609
6.670555776
6.675
6.672391486
6.674899757
6.746157768
7.032597965
7.28520504
7.536384084
7.955000396
8.239452971
8.237931994
8.24
8.24
8.24
8.238564792
8.235
8.238316598
8.24
8.24
8.24
8.239075288
8.239627134
8.238232088
8.239671623
8.24
8.24
8.24
8.238566164
8.24
8.24
8.24
8.24
8.238743261
8.24
8.24
8.24
8.24
8.238260073
6.937517316
3.817757371
2.890021444
2.89174335
2.89262684
2.891899468
2.891966456
2.89216056
2.91649852
7.482917055
8.506906008
8.507382087
8.507260807
8.506755495
8.507626602
8.505
8.508041455
8.506743477
8.506479865
7.0333295
3.203654238
5.030070536
8.441458964
7.766487668
];                % Rampas de cada tramo





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%     Movimiento del vehículo       %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Se declaran las variables y se inician con valor cero 

tiempo=zeros(1,1);
espacio=zeros(1,1);
velocidad=zeros(1,1);
aceleracion=zeros(1,1);
estado=zeros(1,1);
Energia_instant_W_s=zeros(1,1);
Energia_instant_kW_h=zeros(1,1);
Energia_acumulada_motor_kW_h=zeros(1,1);
Fuerza_traccion=zeros(1,1);
Potencia=zeros(1,1);
velocidad_media=zeros(1,1);
Porcentaje_potencia=zeros(1,1);
Rendimiento=zeros(1,1);
Energia_instant_combustible_kW_h=zeros(1,1);
Energia_acumulada_combustible_kW_h=zeros(1,1);
Consumo=zeros(1,1);
Consumo_total=zeros(1,1);

%%%%


paso=2;                         % Inicio del tiempo - Contador general


[ntramos,S]=size(pk_inicio);    % Calcula el número de tramos



% Bucle que recorre de froma sucesiva todos los tramos del recorrido.
% Inicialmente calcula la rampa en radianes, el punto final y la longitud
% de cada tramo

for i=1:(ntramos)
    
if i<ntramos   
    pk_final(i)=pk_inicio(i+1);           % pk fin de tramo
    v_final(i)=v_limite_km_h(i+1)/3.6;    % Velocidad final del tramo (m/s)
                                          % Coincide con la velocidad de límite del tramo siguiente
else
    pk_final(i)=meta;                     % pk_fin del último tramo. Llegada a destino
    v_final(i)=v_limite_km_h(i)/3.6;      % En el último tramo no se indica v_final
end


l_tramo(i)=pk_final(i)-pk_inicio(i) ;     % Longitud del tramo (m)


% Se calcula la cinemática del vehículo en cada paso de simulación

     while (espacio(paso-1) < pk_final(i))
        
                  
        % Cálculo de distancia de frenado 
        d_frenada=(((v_final(i)/3.6)^2-velocidad(paso-1)^2)/(2*ax_brake)); 
        
        
        
        % Determinación del estado (tracción / vel. constante / frenada)
        %   Si está en tracción el estado=1, con lo que a=ax_trac
        %   Si ha llegado a la velocidad máxima del tramo, estado=0. En ese
        %   caso la velocidad es constante y a=0
        %   Si debe empezar la frenada, el estado=-1, con lo que a=ax_brake
        
        
            if velocidad(paso-1)>=v_final(i)  % Condicion de velocidad mayor que la final. 
                                            % En este caso tiene sentido que pueda
                                            % existir frenada.

                if espacio(paso-1)<(pk_final(i)-d_frenada)   % No tiene que frenar, la posición
                                                           % actual indica que no ha llegado al punto de
                                                           % inicio de frenada

                    % Definición de la aceleración en función del estado 
                     
                    if velocidad(paso-1) < (v_limite_km_h(i)/3.6)         % Si no se llega a la velocidad límite se puede acelerar
                         if velocidad(paso-1)<(v1_km_h/3.6)
                             a=ax_trac1;
                         else
                             if velocidad(paso-1)>(v2_km_h/3.6)
                                 a=ax_trac3;
                             else
                                 a=ax_trac2;
                             end
                         end
                         estado(paso)=1;
                    
                         % Si se ha llegado a la velocidad límite del 
                         % tramo, se mantiene la velocidad constante, esto
                         % implica que la aceleración es nula
                     else
                         a=0;
                         estado(paso)=0;
                     end                     % fin if cálculo subtramo tracción   

                else                         % Está en frenada
                   a=ax_brake;
                   estado(paso)=-1;          % Caso de frenado, estado =-1

                end                      % fin if cálculo de inicio de frenada

            else                         % En caso de velocidad(paso)<v_final: No se frena

                    % Definición de la aceleración en función del subtramo 
                     if velocidad(paso-1) < (v_limite_km_h(i)/3.6)         % Si no se llega a la velocidad límite
                         if velocidad(paso-1)<(v1_km_h/3.6)
                             a=ax_trac1;
                         else
                             if velocidad(paso-1)>(v2_km_h/3.6)
                                 a=ax_trac3;
                             else
                                 a=ax_trac2;
                             end
                         end
                                                                         
                         estado(paso)=1;
                                                % Caso de Velocidad limite
                     else
                         a=0;
                         estado(paso)=0;
                     end                     % fin if cálculo subtramo tracción 

            end                          % fin if condición de velocidad superior a final



            aceleracion(paso)=a;       % Se actualiza el valor de Aceleración 
            velocidad(paso)=velocidad(paso-1)+a*delta_t;
            espacio(paso)=espacio(paso-1)+velocidad(paso)*delta_t+0.5*a*(delta_t^2);
            rampa_t(paso)=rampa(i);    % Se asocia un valor de rampa para cada paso
            
            tiempo(paso)=tiempo(paso-1)+delta_t;    % Se actualiza el tiempo          
       
            paso=paso+1;                            % Se actualiza el paso
            
            
     end                                    % Fin del bucle while

end

% % aceleracion(paso)=a; % incluye un último valor para igualar el tamaño
% de los vactores % estado(paso)=estado(paso-1);



tiempo_final=tiempo(paso-1);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%   Cálculo de la potencia y energía %%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Se hace un bucle que recorre todos los pasos, para ello se utiliza el
% valor de la variable "paso"

for j=2:(paso-1)
 
   
    % Se calcula la fuerza en Newtons, utilzando la velocidad en m/s
    if estado(j)==1
        Fuerza_traccion(j)=A+B*velocidad(j)+C*((velocidad(j))^2)+m_t*9.81*0.01*rampa_t(j)+m_t*aceleracion(j);  
    else
            if estado(j)==0
                Fuerza_traccion(j)=A+B*velocidad(j)+C*((velocidad(j))^2)+m_t*9.81*0.01*rampa_t(j);
            else
                % Se simula el caso en el que las resistencias pasivas en
                % llano más la resistencia gravitatoria sean mayores que la
                % aceleración de frenada elegida. En ese caso habría que
                % acelerar y consumir fuerza de tracción
                
                if -(A+B*velocidad(j)+C*((velocidad(j))^2)+m_t*9.81*0.01*rampa_t(j))<ax_brake
                    Fuerza_traccion(j)=A+B*velocidad(j)+C*((velocidad(j))^2)+m_t*9.81*0.01*rampa_t(j)-ax_brake*m_t;
                else
                    Fuerza_traccion(j)=0;  
                end
            end
    end
    
    disp(Fuerza_traccion(j));

    velocidad_media(j)=(velocidad(j)+velocidad(j-1))/2;       % En m/s
    Potencia(j)=Fuerza_traccion(j)*velocidad_media(j);        % En W
    
    disp(velocidad_media(j));
    disp(Potencia(j));
    
    % Energía en cada paso
    Energia_instant_W_s(j)=Potencia(j)*delta_t;               % En W·s
   
    Energia_instant_kW_h(j)=Energia_instant_W_s(j)/3600000;   % En kW·h
    
    % Se calcula la energía acumulada entregada por el motor. Es un valor
    % que identifica el consumo de energía asociado al tramo. Es
    % independiente del motor, pero depende las condiciones dinámicas del
    % vehículo, incluyendo su masa, resistencias al avance...
    
    Energia_acumulada_motor_kW_h(j)=Energia_instant_kW_h(j)+Energia_acumulada_motor_kW_h(j-1);
    
    % Cálculo del consumo de combustible. Para ello se calcula el
    % porcentaje de potencia. Con ese valor se interpola según los valores
    % de rendimiento. Hay que tener en cuenta que la potencia máxima Pmax,
    % está en kW
    
    Porcentaje_potencia(j)=100*Potencia(j)/(Pmax_kW*1000);
    
    
    % Variable intermedia para la función de interpolación. Es el valor
    % buscado
    xq= Porcentaje_potencia(j);

    disp(xq);
    disp("------------");
    if j == 12
        break
    end
    
    Rendimiento(j)=interp1(X_porcentaje,Rendimiento_motor,xq);
    
    
    %Cálculo de la energía instantánea del combustible en función del
    %rendimiento del motor en función de la potencia demandada 
    Energia_instant_combustible_kW_h(j)=Energia_instant_kW_h(j)/(Rendimiento(j)/100);
    
    Energia_acumulada_combustible_kW_h(j)=Energia_instant_combustible_kW_h(j)+Energia_acumulada_combustible_kW_h(j-1);
    
    
    %Conversión de energía del combusible a litros
    Consumo(j)=Energia_instant_combustible_kW_h(j)*l_diesel_por_kWh;
    Consumo_total(j)=Consumo_total(j-1)+Consumo(j);
    
    
    % Valores finales
    Energia_tramo_motor_kW_h=Energia_acumulada_motor_kW_h(j); % Salida de la función kW·h
    Energia_tramo_combustible_kW_h=Energia_acumulada_combustible_kW_h(j); 
    Consumo_tramo_litros=Consumo_total(j);
end
    


%{
figure(1)
plot(tiempo,espacio);                   % Espacio
grid on
xlabel('Tiempo');
ylabel('Espacio');
title('Espacio (m)');
hold off

figure(2)
plot(tiempo,velocidad*3.6);             % Velocidad 
grid on
xlabel('Tiempo');
ylabel('Velocidad (km/h)');
title('Velocidad');
hold off

figure(3)
plot(tiempo,aceleracion);               % Aceleración 
grid on
xlabel('Tiempo');
ylabel('Aceleración');
title('Aceleración');
hold off

figure(4)
plot(tiempo,estado);                    % Estado de tracción o frenado
grid on
xlabel('Tiempo');
ylabel('Estado');
title('Estado');
hold off
%}

