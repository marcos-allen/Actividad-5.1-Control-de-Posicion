clear
close all
clc


%%%%%%%%%%%%%%%%%%%%%%%%%%%% TRAYECTORIA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

trayectoria = [-27 -25 -23 -21 -19 -15 -12 -10 -7 -4 -1 -1 -2 0 1 2 3 3 5 6 8 10 12 13 13 15 19 22 24 26 25 ...
               23 22 19 17 16 15 15 13 11 9 8 7 5 1 0 -4 -6 -8 -10 -11 -12 -13 -14 -17 -18 -22 -24 -25 -27;
               -13 -11 -8 -5 0 -2 -4 -5 -6 -6 -6 -5 1 -1 0 0 1 3 -1 -2 0 4 8 12 16 15 15 15 15 16 14 10 ...
               6 5 3 1 -3 -7 -8 -10 -12 -14 -18 -16 -14 -14 -15 -17 -15 -13 -12 -12 -12 -13 -15 -15 -13 -12 -12 -13]';

num_tramos = size(trayectoria, 1) - 1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% TIEMPO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tf = num_tramos*2;   % Tiempo de simulacion en segundos (s), una rotacion y una traslacion por tramo
ts = 0.1;            % Tiempo de muestreo en segundos (s)
t = 0: ts: tf;       % Vector de tiempo
N = length(t);       % Muestras

%%%%%%%%%%%%%%%%%%%%%%%% CONDICIONES INICIALES %%%%%%%%%%%%%%%%%%%%%%%%%%%%

x1 = zeros (1,N+1);  % Posición en el centro del eje que une las ruedas (eje x) en metros (m)
y1 = zeros (1,N+1);  % Posición en el centro del eje que une las ruedas (eje y) en metros (m)
phi = zeros(1, N+1); % Orientacion del robot en radianes (rad)

x1(1) = -27;    % Posicion inicial eje x
y1(1) = -13;   % Posicion inicial eje y
phi(1) = 0;   % Orientacion inicial del robot

%%%%%%%%%%%%%%%%%%%%%%%%%%%% PUNTO DE CONTROL %%%%%%%%%%%%%%%%%%%%%%%%%%%%

hx = zeros(1, N+1);  % Posicion en el punto de control (eje x) en metros (m)
hy = zeros(1, N+1);  % Posicion en el punto de control (eje y) en metros (m)

hx(1) = x1(1); % Posicion en el punto de control del robot en el eje x
hy(1) = y1(1); % Posicion en el punto de control del robot en el eje y


%%%%%%%%%%%%%%%%%%%%%% VELOCIDADES DE REFERENCIA %%%%%%%%%%%%%%%%%%%%%%%%%%

u = zeros(N, 1); % Velocidad lineal de referencia (m/s)
w = zeros(N, 1); % Velocidad angular de referencia (rad/s)

ultimo_angulo = phi(1);

for k = 1:num_tramos

    dx = trayectoria(k+1, 1) - trayectoria(k, 1);
    dy = trayectoria(k+1, 2) - trayectoria(k, 2);

    distancia = sqrt(dx^2 + dy^2);
    angulo_objetivo = atan2(dy, dx);
    error_angular = angulo_objetivo - ultimo_angulo;
    error_angular = atan2(sin(error_angular), cos(error_angular)); % Normalizar (wrap to pi)

    % Rotacion
    idx_giro = (k-1)*20 + (1:10);
    w(idx_giro) = error_angular;

    % Traslacion
    idx_avance = (k-1)*20 + (11:20);
    u(idx_avance) = distancia;

    ultimo_angulo = angulo_objetivo;
end


%%%%%%%%%%%%%%%%%%%%%%%%% BUCLE DE SIMULACION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for k=1:N 
    
    % Mi phi actual ||| Mi phi hasta el momento anterior
    phi(k+1)=phi(k)+w(k)*ts; % Integral numérica (método de Euler)
    
    %%%%%%%%%%%%%%%%%%%%% MODELO CINEMATICO %%%%%%%%%%%%%%%%%%%%%%%%%
    %Aplicamos el modelo cinemático diferencial para obtener las
    %velocidades en x, y, phi
    xp1=u(k)*cos(phi(k+1)); 
    yp1=u(k)*sin(phi(k+1));
    phip = w(k);

    x1(k+1)=x1(k) + xp1*ts ; % Integral numérica (método de Euler)
    y1(k+1)=y1(k) + yp1*ts ; % Integral numérica (método de Euler)
    

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
set(scene,'position',sizeScreen); % Congigurar tamaño de la figura
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
H2=plot3(hx(1),hy(1),0,'g','lineWidth',2);

% d) Bucle de simulacion de movimiento del robot

step=1; % pasos para simulacion

for k=1:step:N

    delete(H1);    
    delete(H2);
    
    H1=MobilePlot_4(x1(k),y1(k),phi(k),scale);
    H2=plot3(hx(1:k),hy(1:k),zeros(1,k),'g','lineWidth',2);
    
    pause(ts);

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PUZZLEBOT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

r = 0.05;            % Radio de las ruedas (m)
l = 0.17;            % Eje / Distancia entre ruedas (m)

% Calcular las velocidades de las ruedas a partir de las velocidades lineales y angulares
w_r = (u + (l/2) * w) / r; % Velocidad de la rueda derecha
w_l = (u - (l/2) * w) / r; % Velocidad de la rueda izquierda


%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Graficas %%%%%%%%%%%%%%%%%%%%%%%%%%%%
graph=figure;  % Crear figura (Escena)
set(graph,'position',sizeScreen); % Congigurar tamaño de la figura
subplot(711)
plot(t,u,color='#FF0000',LineWidth=2),grid('on'),xlabel('Tiempo [s]'),ylabel('v [m/s]');
subplot(712)
plot(t,w,color='#FF7F00',LineWidth=2),grid('on'),xlabel('Tiempo [s]'),ylabel('\omega [rad/s]');
subplot(713)
plot(t,x1(1:N),color='#FFFF00',LineWidth=2),grid('on'),xlabel('Tiempo [s]'),ylabel('x [m]');
subplot(714)
plot(t,y1(1:N),color='#00FF00',LineWidth=2),grid('on'),xlabel('Tiempo [s]'),ylabel('y [m]');
subplot(715)
plot(t,phi(1:N),color='#0000FF',LineWidth=2),grid('on'),xlabel('Tiempo [s]'),ylabel('\theta [rad]');
subplot(716)
plot(t,w_r,color='#9400D3',LineWidth=2),grid('on'),xlabel('Tiempo [s]'),ylabel('\omega_R [rad/s]');
subplot(717)
plot(t,w_l,color='#FF00FF',LineWidth=2),grid('on'),xlabel('Tiempo [s]'),ylabel('\omega_L [rad/s]');
