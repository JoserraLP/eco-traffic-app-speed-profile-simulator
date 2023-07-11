clear
clc

format longG

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%          Datos del veh�culo         %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

m_v=1400;         % Masa del veh�culo en vac�o en kg
m_carga=300;      % Masa de las carga del veh�culo en kg
m_t=m_v+m_carga;  % Masa total del veh�culo
  
gamma=1.05;       % Factor de mayoraci�n de masas rotativas

%Resistencias al avance R=A+B�v(t)+C�v^2�(t)+m_t�g�sin?
fr=0.015;         % Coeficiente de resistencias al avance 
A= m_t*fr*9.81;   % Par�metro en N
B= 0;             % Par�mentro en N/(m/s)  

% Efecto aerodin�mico
Af=2;             % �rea frontal en metros
ro=1.225;         % Densidad del aire en kg/m^3
Cx=0.3;           % Coeficiente aerodin�mico

C=0.5*ro*Af*Cx;   % Valor del par�metro C, funci�n de la v^2. Expresado en N/(m/s)^2

% Si se disponen de otros valores m�s precisos, se pueden introducir
% directamente para el c�lculo de las resistencias pasivas, Cuidado con las
% unidades, ya que habitualmente suelen estar expresados para una velocidad
% en km/h y no en m/s


% Definici�n de los criterios de conducci�n. Se establecen tres franjas de conducci�n 

v1_km_h=50;      % Primer l�mite de velocidad en km/h
v2_km_h=100;     % Segundo l�mite de velocidad en km/h

ax_trac1=2;      % Aceleraci�n longitudinal de tracci�n m�xima en m/s^2 con velocidad menor que v1_km_h
ax_trac2=1;      % Aceleraci�n longitudinal de tracci�n m�xima en m/s^2 con velocidad entre v1_km_h y v2_km_h
ax_trac3=0.5;    % Aceleraci�n longitudinal de tracci�n m�xima en m/s^2 con velocidad mayor que v2_km_h

ax_brake=-2;     % Aceleraci�n longitudinal de frenada en m/s^2 

delta_t=0.1;     % Incremento de tiempo entre pasos

Pmax_kW=100;        % Potencia m�xima en kW del motor

% Valor de porcentaje de la pontencia m�xima para el c�lculo del
% rendimiento seg�n FASTSIM
X_porcentaje=[0 0.5 1.5 4 6 10 14 20 40 60 80 100]; 

% Valor del rendimiento seg�n FASTSIM
Rendimiento_motor=[10 14 20 26 32 39 41 42 41 38 36 34];

kWh_por_l_diesel=10;                 % Conversi�n de litros gas�leo a kWh 
l_diesel_por_kWh=1/kWh_por_l_diesel; % Inverso del anterior para obtener litros de gas�leo por kWh

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% Definici�n del trayecto %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% En esta parte se introducen los datos que configuran el recorrido a
% analizar. El trayecto se define por una sucesi�n de tramos. Cada tramo
% est� definido por las siguientes matrices:
%   pk_inicio:     Variable que indica el punto de inicio de cada tramo, 
%                  medida en metros desde el origen.
%   v_limite_km_h: Velocidad l�mite de cada tramo, expresada en km/h
%   rampa:         Rampa del tramo, expresada en grados sexagesimanles 





meta=5260;                        % Longitud total del recorrido

% La definici�n del recorrido se caracteriza por la distancia al origen del
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


% Velocidades l�mite en cada tramo, obtenido de la m�xima del tramo,
% modificada por el estado del tr�fico. Est� expresada en km/h

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
% rampa(%)=100*tan(inclinaci�n)=100*seno(inclinaci�n)

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
%%%%%%%%%%%%%%%%%%%%%     Movimiento del veh�culo       %%%%%%%%%%%%%%%%%%%
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


[ntramos,S]=size(pk_inicio);    % Calcula el n�mero de tramos



% Bucle que recorre de froma sucesiva todos los tramos del recorrido.
% Inicialmente calcula la rampa en radianes, el punto final y la longitud
% de cada tramo

for i=1:(ntramos)
    
if i<ntramos   
    pk_final(i)=pk_inicio(i+1);           % pk fin de tramo
    v_final(i)=v_limite_km_h(i+1)/3.6;    % Velocidad final del tramo (m/s)
                                          % Coincide con la velocidad de l�mite del tramo siguiente
else
    pk_final(i)=meta;                     % pk_fin del �ltimo tramo. Llegada a destino
    v_final(i)=v_limite_km_h(i)/3.6;      % En el �ltimo tramo no se indica v_final
end


l_tramo(i)=pk_final(i)-pk_inicio(i) ;     % Longitud del tramo (m)


% Se calcula la cinem�tica del veh�culo en cada paso de simulaci�n

     while (espacio(paso-1) < pk_final(i))
        
                  
        % C�lculo de distancia de frenado 
        d_frenada=(((v_final(i)/3.6)^2-velocidad(paso-1)^2)/(2*ax_brake)); 
        
        
        
        % Determinaci�n del estado (tracci�n / vel. constante / frenada)
        %   Si est� en tracci�n el estado=1, con lo que a=ax_trac
        %   Si ha llegado a la velocidad m�xima del tramo, estado=0. En ese
        %   caso la velocidad es constante y a=0
        %   Si debe empezar la frenada, el estado=-1, con lo que a=ax_brake
        
        
            if velocidad(paso-1)>=v_final(i)  % Condicion de velocidad mayor que la final. 
                                            % En este caso tiene sentido que pueda
                                            % existir frenada.

                if espacio(paso-1)<(pk_final(i)-d_frenada)   % No tiene que frenar, la posici�n
                                                           % actual indica que no ha llegado al punto de
                                                           % inicio de frenada

                    % Definici�n de la aceleraci�n en funci�n del estado 
                     
                    if velocidad(paso-1) < (v_limite_km_h(i)/3.6)         % Si no se llega a la velocidad l�mite se puede acelerar
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
                    
                         % Si se ha llegado a la velocidad l�mite del 
                         % tramo, se mantiene la velocidad constante, esto
                         % implica que la aceleraci�n es nula
                     else
                         a=0;
                         estado(paso)=0;
                     end                     % fin if c�lculo subtramo tracci�n   

                else                         % Est� en frenada
                   a=ax_brake;
                   estado(paso)=-1;          % Caso de frenado, estado =-1

                end                      % fin if c�lculo de inicio de frenada

            else                         % En caso de velocidad(paso)<v_final: No se frena

                    % Definici�n de la aceleraci�n en funci�n del subtramo 
                     if velocidad(paso-1) < (v_limite_km_h(i)/3.6)         % Si no se llega a la velocidad l�mite
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
                     end                     % fin if c�lculo subtramo tracci�n 

            end                          % fin if condici�n de velocidad superior a final



            aceleracion(paso)=a;       % Se actualiza el valor de Aceleraci�n 
            velocidad(paso)=velocidad(paso-1)+a*delta_t;
            espacio(paso)=espacio(paso-1)+velocidad(paso)*delta_t+0.5*a*(delta_t^2);
            rampa_t(paso)=rampa(i);    % Se asocia un valor de rampa para cada paso
            
            tiempo(paso)=tiempo(paso-1)+delta_t;    % Se actualiza el tiempo          
       
            paso=paso+1;                            % Se actualiza el paso
            
            
     end                                    % Fin del bucle while

end

% % aceleracion(paso)=a; % incluye un �ltimo valor para igualar el tama�o
% de los vactores % estado(paso)=estado(paso-1);



tiempo_final=tiempo(paso-1);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%   C�lculo de la potencia y energ�a %%%%%%%%%%%%%%%%%%%%%%%%%%
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
                % llano m�s la resistencia gravitatoria sean mayores que la
                % aceleraci�n de frenada elegida. En ese caso habr�a que
                % acelerar y consumir fuerza de tracci�n
                
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
    
    % Energ�a en cada paso
    Energia_instant_W_s(j)=Potencia(j)*delta_t;               % En W�s
   
    Energia_instant_kW_h(j)=Energia_instant_W_s(j)/3600000;   % En kW�h
    
    % Se calcula la energ�a acumulada entregada por el motor. Es un valor
    % que identifica el consumo de energ�a asociado al tramo. Es
    % independiente del motor, pero depende las condiciones din�micas del
    % veh�culo, incluyendo su masa, resistencias al avance...
    
    Energia_acumulada_motor_kW_h(j)=Energia_instant_kW_h(j)+Energia_acumulada_motor_kW_h(j-1);
    
    % C�lculo del consumo de combustible. Para ello se calcula el
    % porcentaje de potencia. Con ese valor se interpola seg�n los valores
    % de rendimiento. Hay que tener en cuenta que la potencia m�xima Pmax,
    % est� en kW
    
    Porcentaje_potencia(j)=100*Potencia(j)/(Pmax_kW*1000);
    
    
    % Variable intermedia para la funci�n de interpolaci�n. Es el valor
    % buscado
    xq= Porcentaje_potencia(j);

    disp(xq);
    disp("------------");
    if j == 12
        break
    end
    
    Rendimiento(j)=interp1(X_porcentaje,Rendimiento_motor,xq);
    
    
    %C�lculo de la energ�a instant�nea del combustible en funci�n del
    %rendimiento del motor en funci�n de la potencia demandada 
    Energia_instant_combustible_kW_h(j)=Energia_instant_kW_h(j)/(Rendimiento(j)/100);
    
    Energia_acumulada_combustible_kW_h(j)=Energia_instant_combustible_kW_h(j)+Energia_acumulada_combustible_kW_h(j-1);
    
    
    %Conversi�n de energ�a del combusible a litros
    Consumo(j)=Energia_instant_combustible_kW_h(j)*l_diesel_por_kWh;
    Consumo_total(j)=Consumo_total(j-1)+Consumo(j);
    
    
    % Valores finales
    Energia_tramo_motor_kW_h=Energia_acumulada_motor_kW_h(j); % Salida de la funci�n kW�h
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
plot(tiempo,aceleracion);               % Aceleraci�n 
grid on
xlabel('Tiempo');
ylabel('Aceleraci�n');
title('Aceleraci�n');
hold off

figure(4)
plot(tiempo,estado);                    % Estado de tracci�n o frenado
grid on
xlabel('Tiempo');
ylabel('Estado');
title('Estado');
hold off
%}

