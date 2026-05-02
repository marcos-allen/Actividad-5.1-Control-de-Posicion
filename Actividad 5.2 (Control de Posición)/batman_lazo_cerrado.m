%Limpieza de pantalla
clear all
close all
clc

%1 TIEMPO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tf=150;            % Tiempo de simulación en segundos (s)
ts=0.1;            % Tiempo de muestreo en segundos (s)
t=0:ts:tf;         % Vector de tiempo
N= length(t);      % Muestras


%2 CONDICIONES INICIALES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Damos valores a nuestro punto inicial de posición y orientación
x1(1)=-27;  %Posición inicial eje x
y1(1)=-13;  %Posición inicial eje y
phi(1)=atan2(-11--13,-25--27); %Orientación inicial del robot 

%3 POSICION DESEADA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Damos los puntos de la trayectoria deseada
hxd = [-27 -25 -23 -21 -19 -15 -12 -10 -7 -4 -1 -1 -2 0 1 2 3 3 5 6 8 10 12 13 13 15 19 22 24 26 25 ...
               23 22 19 17 16 15 15 13 11 9 8 7 5 1 0 -4 -6 -8 -10 -11 -12 -13 -14 -17 -18 -22 -24 -25 -27];
hyd = [-13 -11 -8 -5 0 -2 -4 -5 -6 -6 -6 -5 1 -1 0 0 1 3 -1 -2 0 4 8 12 16 15 15 15 15 16 14 10 ...
               6 5 3 1 -3 -7 -8 -10 -12 -14 -18 -16 -14 -14 -15 -17 -15 -13 -12 -12 -12 -13 -15 -15 -13 -12 -12 -13];
num_pts = length(hxd);  
pt_act = 1;          % Se inicia en el primer punto
umbral = 0.25;       % Umbral del error para poder cambiar al siguiente punto

%Igualamos el punto de control con las proyecciones X1 y Y1 por su
%coincidencia
hx(1)= x1(1);       % Posición del punto de control en el eje (X) metros (m)
hy(1)= y1(1);       % Posición del punto de control en el eje (Y) metros (m)

%4 CONTROL, BUCLE DE SIMULACION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Ganancias para el controlador auto-sintonizable
kMax = 2;    % Ganancia maxima
kMin = 0.5;  % Ganancia minima

alpha = 0.25; % Factor de suavizacion
k_auto_prev = kMin;

for k=1:N
    
    if pt_act <= num_pts
        %Errores de control
        hxe(k)=hxd(pt_act)-hx(k);
        hye(k)=hyd(pt_act)-hy(k);
        
        %Matriz de error
        he= [hxe(k);hye(k)];
        %Magnitud del error de posición
        Error(k)= sqrt(hxe(k)^2 +hye(k)^2);
        
        %Verificamos si el error es menor que el umbral para cambiar al siguiente punto
        if Error(k) < umbral && pt_act < num_pts
            pt_act = pt_act + 1; %Avanzamos al siguiente punto de control
            %Recalculamos el error con el siguiente punto
            hxe(k)=hxd(pt_act)-hx(k);
            hye(k)=hyd(pt_act)-hy(k);
            he= [hxe(k);hye(k)];
            Error(k)= sqrt(hxe(k)^2 +hye(k)^2);
        end
    
        %Calculamos la ganancia con base en el error actual
        %Cuando el error es grande la parte exponencial tiende a 0, y se
        %obtiene la ganancia máxima, y cuando el error es pequeño la parte
        %exponencial tiende a 1 y se obtiene la ganancia mínima
        k_auto = kMin + (kMax - kMin) * (1 - exp(-Error(k)));
        
        %Se aplica el filtro EMA para obtener una respuesta rapida o suave,
        %dependiendo de lo que se desee
        k_auto = (1-alpha) * k_auto_prev + alpha * k_auto;
        k_auto_prev = k_auto;
    
        %Matriz de Ganancias auto-sintonizable
        K=[k_auto 0;...
           0 k_auto];
    
        %Matriz Jacobiana
        J=[cos(phi(k)) -sin(phi(k));... %Matriz de rotación en 2D
           sin(phi(k)) cos(phi(k))];
    
        %Ley de Control
        qpRef= pinv(J)*K*he;
    
        v(k)= qpRef(1);   %Velocidad lineal de entrada al robot 
        w(k)= qpRef(2);   %Velocidad angular de entrada al robot
    
    else
        % Si se llegó al punto final, el robot se detiene
        Error(k) = 0;
        v(k) = 0;
        w(k) = 0;
    end


%5 APLICACIÓN DE CONTROL AL ROBOT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %Aplico la integral a la velocidad angular para obtener el angulo "phi" de la orientación
    phi(k+1)=phi(k)+w(k)*ts; % Integral numérica (método de Euler)
           
   %%%%%%%%%%%%%%%%%%%%% MODELO CINEMATICO %%%%%%%%%%%%%%%%%%%%%%%%%
    
    xp1=v(k)*cos(phi(k)); 
    yp1=v(k)*sin(phi(k));
 
    %Aplico la integral a la velocidad lineal para obtener las cordenadas
    %"x1" y "y1" de la posición
    x1(k+1)=x1(k)+ xp1*ts; % Integral numérica (método de Euler)
    y1(k+1)=y1(k)+ yp1*ts; % Integral numérica (método de Euler)

    % Posicion del robot con respecto al punto de control
    hx(k+1)=x1(k+1); 
    hy(k+1)=y1(k+1);
     

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULACION VIRTUAL 3D %%%%%%%%%%%%%%%%%%%%%%%%%%%%

% a) Configuracion de escena

scene=figure;  % Crear figura (Escena)
set(scene,'Color','white'); % Color del fondo de la escena
set(gca,'FontWeight','bold') ;% Negrilla en los ejes y etiquetas
sizeScreen=get(0,'ScreenSize'); % Retorna el tamaño de la pantalla del computador
set(scene,'position',sizeScreen); % Configurar tamaño de la figura
camlight('headlight'); % Luz para la escena
axis equal; % Establece la relación de aspecto para que las unidades de datos sean las mismas en todas las direcciones.
grid on; % Mostrar líneas de cuadrícula en los ejes
box on; % Mostrar contorno de ejes
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)'); % Etiqueta de los eje

view([25 25]); % Orientacion de la figura
axis([-30 30 -20 20 0 2]); % Ingresar limites minimos y maximos en los ejes x y z [minX maxX minY maxY minZ maxZ]

% b) Graficar robots en la posicion inicial
scale = 4;
MobileRobot_5;
H1=MobilePlot_4(x1(1),y1(1),phi(1),scale);hold on;

% c) Graficar Trayectorias
H2=plot3(hx(1),hy(1),0,'r','lineWidth',2);
H3=plot3(hxd,hyd,zeros(size(hxd)),'bo','lineWidth',2); %Grafico circulo en posición deseada
H4=plot3(hx(1),hy(1),0,'go','lineWidth',2);%Grafico circulo en posición inicial
% d) Bucle de simulacion de movimiento del robot

step=1; % pasos para simulacion

for k=1:step:N

    delete(H1);    
    delete(H2);
    
    H1=MobilePlot_4(x1(k),y1(k),phi(k),scale);
    H2=plot3(hx(1:k),hy(1:k),zeros(1,k),'r','lineWidth',2);
    
    pause(ts);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Graficas %%%%%%%%%%%%%%%%%%%%%%%%%%%%
graph=figure;  % Crear figura (Escena)
set(graph,'position',sizeScreen); % Congigurar tamaño de la figura
subplot(311)
plot(t,v,'b','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('m/s'),legend('Velocidad Lineal (v)');
subplot(312)
plot(t,w,'g','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('[rad/s]'),legend('Velocidad Angular (w)');
subplot(313)
plot(t,Error,'r','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('[metros]'),legend('Error de posición (m)');

