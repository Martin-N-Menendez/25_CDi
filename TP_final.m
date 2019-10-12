

%----------------------- Inicializaci�n -----------------------------------

clear all;
%close all;
clc;

%------------------- Parametros electronicos ------------------------------

R = 10e3;       % 10 KOhm
C = 1e-6;       %  1 uF

h = 3e-3;        %  3 ms

%------------------- Discretizaci�n de la planta --------------------------

h = 3e-3;        %  3 ms

matriz_a = exp(-h/(R*C));
matriz_b = C*( 1 - exp(-h/(R*C)) );
matriz_c = 1/C;
matriz_d = 0;

% Calcular numearador y denominador
[Numerador Denominador] = ss2tf(matriz_a, matriz_b, matriz_c, matriz_d, 1)

% Obtener transferencia del sistema
H = tf(Numerador, Denominador, h)

% Calcular ceros del sistema
z = zero(H)
% Calcular polos del sistema
p = pole(H)

%------------- Ploteo de los resultados a lazo abierto --------------------

% Respuesta al impulso
figure;
impulse(H);

% Respuesta al escal�n unitario
figure;
step(H);

% Diagrama de polos/ceros
figure;
zplane(z,p);
title('Diagrama de Polos y Ceros');

% Diagrama de bode
figure
bode(H)

%------------------------ Ploteo de la referencia -------------------------


referencia =  [2.15;2.15;2.15;2.15;2.15;2.15;2.15;2.15;2.15;2.15;1.15;1.15;1.15;1.15;1.15;1.15;1.15;1.15;1.15;1.15;2.15;2.15;2.15;2.15;2.15;2.15;2.15;2.15;2.15;2.15;1.15;1.15;1.15;1.15;1.15;1.15;1.15;1.15;1.15;1.15;2.15;2.15;2.15;2.15;2.15;2.15;2.15;2.15;2.15;2.15;1.15;1.15;1.15;1.15;1.15;1.15;1.15;1.15;1.15;1.15;2.15;2.15;2.15;2.15;2.15;2.15;2.15;2.15;2.15;2.15;1.15;1.15;1.15;1.15;1.15;1.15;1.15;1.15;1.15;1.15;2.15;2.15;2.15;2.15;2.15;2.15;2.15;2.15;2.15;2.15;1.15;1.15;1.15;1.15;1.15;1.15;1.15;1.15;1.15;1.15;2.15;2.15;2.15;2.15;2.15;2.15;2.15;2.15;2.15;2.15;1.15;1.15;1.15;1.15;1.15;1.15;1.15;1.15;1.15;1.15;2.15;2.15;2.15;2.15;2.15;2.15;2.15;2.15;2.15;2.15;1.15;1.15;1.15;1.15;1.15;1.15;1.15;1.15;1.15;1.15;2.15;2.15;2.15;2.15;2.15;2.15;2.15;2.15;2.15;2.15;1.15;1.15;1.15;1.15;1.15;1.15;1.15;1.15;1.15;1.15;2.15;2.15;2.15;2.15;2.15;2.15;2.15;2.15;2.15;2.15;1.15;1.15;1.15;1.15;1.15;1.15;1.15;1.15;1.15;1.15;2.15;2.15;2.15;2.15;2.15;2.15;2.15;2.15;2.15;2.15;1.15;1.15;1.15;1.15;1.15;1.15;1.15;1.15;1.15;1.15];

tiempo = linspace(0,1,200);

figure
plot(tiempo,referencia)
title('Referencia generada en EDU-CIAA')
xlabel('tiempo[segundos]') 
ylabel('Amplitud[volts]') 

%------------------------ Ploteo sin control -------------------------

clf;

R = [2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 ];

Y = [0.00 0.62 1.05 1.37 1.59 1.75 1.86 1.95 2.00 2.04 2.07 1.80 1.62 1.48 1.39 1.32 1.27 1.24 1.21 1.20 1.18 1.46 1.66 1.80 1.90 1.97 2.02 2.06 2.08 2.10 2.12 1.84 1.64 1.50 1.40 1.33 1.28 1.24 1.22 1.20 1.19 1.47 1.66 1.80 1.90 1.97 2.02 2.06 2.09 2.10 2.12 1.84 1.64 1.50 1.40 1.33 1.28 1.24 1.22 1.20 1.19 1.47 1.66 1.80 1.90 1.97 2.02 2.06 2.09 2.10 2.12 1.84 1.64 1.50 1.40 1.33 1.28 1.25 1.22 1.20 1.19 1.47 1.66 1.81 1.90 1.97 2.03 2.06 2.09 2.10 2.12 1.84 1.64 1.50 1.40 1.33 1.28 1.25 1.22 1.20 1.19 1.47 1.66 1.81 1.90 1.97 2.03 2.06 2.09 2.10 2.12 1.84 1.64 1.50 1.40 1.33 1.28 1.25 1.22 1.20 1.19 1.47 1.66 1.81 1.90 1.97 2.02 2.06 2.08 2.10 2.12 1.84 1.64 1.50 1.40 1.33 1.28 1.24 1.22 1.20 1.19 1.47 1.66 1.81 1.90 1.97 2.03 2.06 2.09 2.10 2.12 1.84 1.64 1.50 1.40 1.33 1.28 1.25 1.22 1.20 1.19 1.47 1.66 1.81 1.90 1.97 2.03 2.06 2.09 2.11 2.12 1.84 1.64 1.50 1.40 1.33 1.28 1.25 1.22 1.20 1.19 1.47 1.66 1.81 1.90 1.97 2.03 2.06 2.09 2.11 2.12 1.84 1.64 1.50 1.40 1.33 1.28 1.25 1.22 1.20 ];

U = [2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 ];

Usat = [2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 ];


tiempo = linspace(0,1,200);

figure(6)
hold on
plot(tiempo,R)
plot(tiempo,Y)
%plot(tiempo,U)
%plot(tiempo,Usat)
title('Seguimiento de la referencia sin control')
xlabel('tiempo[segundos]') 
ylabel('Amplitud[volts]') 
legend('Referencia','Salida','Control','Control_{sat}')

%-------------------------- Ploteo con PID  -------------------------------

clf;

R = [2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 ];

Y = [0.00 0.74 1.31 1.76 2.10 2.37 2.57 2.74 2.78 2.64 2.47 1.91 1.61 1.38 1.23 1.15 1.12 1.12 1.12 1.13 1.14 1.63 2.01 2.28 2.35 2.33 2.27 2.23 2.19 2.16 2.15 1.66 1.45 1.29 1.19 1.14 1.12 1.12 1.13 1.14 1.14 1.64 2.01 2.28 2.35 2.33 2.28 2.22 2.19 2.16 2.15 1.66 1.45 1.29 1.19 1.14 1.12 1.12 1.13 1.14 1.14 1.64 2.01 2.28 2.35 2.33 2.28 2.23 2.18 2.16 2.15 1.66 1.45 1.29 1.19 1.14 1.13 1.12 1.13 1.14 1.14 1.63 2.01 2.28 2.35 2.33 2.28 2.22 2.19 2.16 2.15 1.66 1.45 1.29 1.19 1.14 1.13 1.12 1.13 1.14 1.14 1.63 2.01 2.27 2.35 2.33 2.28 2.22 2.19 2.16 2.15 1.66 1.45 1.28 1.19 1.14 1.13 1.12 1.13 1.14 1.14 1.63 2.01 2.28 2.35 2.33 2.28 2.22 2.19 2.16 2.15 1.66 1.45 1.28 1.19 1.14 1.12 1.13 1.13 1.14 1.14 1.63 2.01 2.28 2.35 2.33 2.28 2.22 2.19 2.16 2.15 1.66 1.45 1.29 1.19 1.14 1.12 1.12 1.13 1.14 1.14 1.63 2.01 2.28 2.35 2.33 2.28 2.05 2.16 2.20 2.20 1.70 1.49 1.32 1.21 1.15 1.13 1.12 1.13 1.14 1.14 1.63 2.01 2.27 2.35 2.33 2.28 2.22 2.19 2.16 2.15 1.66 1.45 1.28 1.19 1.14 1.13 1.12 1.13 1.14 ];

U = [6.02 6.31 6.25 5.92 5.39 4.69 3.87 2.95 2.18 1.88 -0.98 0.58 0.58 0.72 0.88 1.01 1.09 1.13 1.16 1.16 3.96 3.69 3.21 2.61 2.26 2.11 2.06 2.05 2.08 2.11 -0.67 0.69 0.73 0.85 0.97 1.06 1.12 1.15 1.16 1.16 3.96 3.68 3.20 2.60 2.27 2.11 2.05 2.06 2.08 2.11 -0.68 0.68 0.73 0.85 0.96 1.06 1.12 1.15 1.16 1.16 3.96 3.68 3.20 2.60 2.27 2.11 2.05 2.05 2.08 2.11 -0.67 0.69 0.73 0.85 0.97 1.06 1.11 1.14 1.15 1.16 3.96 3.69 3.20 2.61 2.26 2.11 2.05 2.06 2.08 2.11 -0.68 0.69 0.73 0.85 0.97 1.06 1.11 1.14 1.15 1.16 3.95 3.69 3.20 2.61 2.26 2.11 2.05 2.06 2.08 2.10 -0.67 0.69 0.73 0.86 0.97 1.06 1.11 1.14 1.15 1.16 3.96 3.69 3.20 2.61 2.26 2.11 2.05 2.06 2.08 2.10 -0.67 0.69 0.73 0.86 0.97 1.06 1.12 1.14 1.15 1.16 3.96 3.69 3.20 2.61 2.26 2.11 2.05 2.06 2.08 2.11 -0.68 0.68 0.73 0.85 0.96 1.05 1.12 1.14 1.15 1.16 3.96 3.69 3.20 2.61 2.26 2.11 2.05 2.53 2.33 2.21 -0.66 0.74 0.74 0.83 0.94 1.04 1.10 1.14 1.15 1.15 3.96 3.69 3.20 2.61 2.26 2.11 2.05 2.06 2.08 2.10 -0.67 0.69 0.73 0.86 0.97 1.06 1.11 1.14 1.15 1.16 ];

Usat = [3.30 3.30 3.30 3.30 3.30 3.30 3.30 2.95 2.18 1.88 0.00 0.58 0.58 0.72 0.88 1.01 1.09 1.13 1.16 1.16 3.30 3.30 3.21 2.61 2.26 2.11 2.06 2.05 2.08 2.11 0.00 0.69 0.73 0.85 0.97 1.06 1.12 1.15 1.16 1.16 3.30 3.30 3.20 2.60 2.27 2.11 2.05 2.06 2.08 2.11 0.00 0.68 0.73 0.85 0.96 1.06 1.12 1.15 1.16 1.16 3.30 3.30 3.20 2.60 2.27 2.11 2.05 2.05 2.08 2.11 0.00 0.69 0.73 0.85 0.97 1.06 1.11 1.14 1.15 1.16 3.30 3.30 3.20 2.61 2.26 2.11 2.05 2.06 2.08 2.11 0.00 0.69 0.73 0.85 0.97 1.06 1.11 1.14 1.15 1.16 3.30 3.30 3.20 2.61 2.26 2.11 2.05 2.06 2.08 2.10 0.00 0.69 0.73 0.86 0.97 1.06 1.11 1.14 1.15 1.16 3.30 3.30 3.20 2.61 2.26 2.11 2.05 2.06 2.08 2.10 0.00 0.69 0.73 0.86 0.97 1.06 1.12 1.14 1.15 1.16 3.30 3.30 3.20 2.61 2.26 2.11 2.05 2.06 2.08 2.11 0.00 0.68 0.73 0.85 0.96 1.05 1.12 1.14 1.15 1.16 3.30 3.30 3.20 2.61 2.26 2.11 2.05 2.53 2.33 2.21 0.00 0.74 0.74 0.83 0.94 1.04 1.10 1.14 1.15 1.15 3.30 3.30 3.20 2.61 2.26 2.11 2.05 2.06 2.08 2.10 0.00 0.69 0.73 0.86 0.97 1.06 1.11 1.14 1.15 1.16 ];


tiempo = linspace(0,1,200);

figure(7)
hold on
plot(tiempo,R)
plot(tiempo,Y)
%plot(tiempo,U)
plot(tiempo,Usat)
title('Seguimiento de la referencia con control PID')
xlabel('tiempo[segundos]') 
ylabel('Amplitud[volts]') 
legend('Referencia','Salida','Control','Control_{sat}')

%-------------------- Ploteo con Pole placement  --------------------------

clf;

R = [2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 2.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 1.15 ];

Y = [0.00 0.59 1.20 1.61 1.84 1.97 2.05 2.09 2.11 2.13 2.14 1.70 1.46 1.33 1.25 1.21 1.18 1.17 1.16 1.15 1.15 1.59 1.83 1.97 2.05 2.09 2.11 2.13 2.14 2.14 2.15 1.71 1.46 1.33 1.25 1.21 1.18 1.17 1.16 1.16 1.15 1.59 1.83 1.97 2.05 2.09 2.11 2.13 2.14 2.14 2.15 1.71 1.46 1.33 1.25 1.21 1.18 1.17 1.16 1.16 1.15 1.59 1.83 1.97 2.05 2.09 2.11 2.13 2.14 2.14 2.15 1.71 1.46 1.33 1.25 1.21 1.18 1.17 1.16 1.16 1.15 1.59 1.83 1.97 2.05 2.09 2.11 2.13 2.14 2.14 2.15 1.71 1.47 1.33 1.25 1.21 1.18 1.17 1.16 1.16 1.15 1.59 1.84 1.97 2.05 2.09 2.11 2.13 2.14 2.14 2.15 1.71 1.46 1.33 1.25 1.21 1.18 1.17 1.16 1.16 1.15 1.59 1.83 1.97 2.05 2.09 2.11 2.13 2.14 2.14 2.15 1.71 1.46 1.33 1.25 1.21 1.18 1.17 1.16 1.16 1.15 1.59 1.83 1.97 2.05 2.09 2.11 2.13 2.14 2.14 2.15 1.71 1.46 1.33 1.25 1.21 1.18 1.17 1.16 1.16 1.15 1.59 1.84 1.96 2.04 2.09 2.11 2.13 2.14 2.14 2.15 1.71 1.47 1.33 1.25 1.21 1.18 1.17 1.16 1.16 1.15 1.59 1.84 1.97 2.05 2.09 2.11 2.13 2.14 2.14 2.15 1.71 1.46 1.33 1.25 1.21 1.18 1.17 1.16 1.16 ];

U = [4.14 3.60 3.03 2.65 2.43 2.31 2.24 2.21 2.18 2.17 0.23 0.63 0.86 0.98 1.05 1.09 1.12 1.13 1.14 1.14 3.07 2.66 2.44 2.31 2.24 2.20 2.18 2.17 2.16 2.15 0.22 0.63 0.85 0.98 1.05 1.09 1.12 1.13 1.14 1.14 3.07 2.66 2.44 2.31 2.24 2.20 2.18 2.17 2.16 2.15 0.22 0.63 0.85 0.98 1.05 1.09 1.12 1.13 1.14 1.14 3.07 2.66 2.44 2.31 2.24 2.20 2.18 2.17 2.16 2.15 0.22 0.63 0.85 0.98 1.05 1.09 1.12 1.13 1.14 1.14 3.07 2.66 2.44 2.31 2.24 2.20 2.18 2.17 2.16 2.15 0.22 0.63 0.85 0.98 1.05 1.09 1.12 1.13 1.14 1.14 3.07 2.66 2.44 2.32 2.24 2.20 2.18 2.17 2.16 2.15 0.22 0.63 0.85 0.98 1.05 1.09 1.12 1.13 1.14 1.14 3.07 2.66 2.44 2.31 2.24 2.20 2.18 2.17 2.16 2.15 0.22 0.63 0.85 0.98 1.05 1.09 1.12 1.13 1.14 1.14 3.07 2.66 2.44 2.31 2.24 2.20 2.18 2.17 2.16 2.15 0.22 0.63 0.85 0.98 1.05 1.09 1.12 1.13 1.14 1.14 3.07 2.66 2.44 2.32 2.25 2.21 2.18 2.17 2.16 2.15 0.22 0.63 0.85 0.98 1.05 1.09 1.12 1.13 1.14 1.14 3.07 2.66 2.44 2.32 2.24 2.20 2.18 2.17 2.16 2.15 0.22 0.63 0.85 0.98 1.05 1.09 1.12 1.13 1.14 1.14 ];

Usat = [3.30 3.30 3.03 2.65 2.43 2.31 2.24 2.21 2.18 2.17 0.23 0.63 0.86 0.98 1.05 1.09 1.12 1.13 1.14 1.14 3.07 2.66 2.44 2.31 2.24 2.20 2.18 2.17 2.16 2.15 0.22 0.63 0.85 0.98 1.05 1.09 1.12 1.13 1.14 1.14 3.07 2.66 2.44 2.31 2.24 2.20 2.18 2.17 2.16 2.15 0.22 0.63 0.85 0.98 1.05 1.09 1.12 1.13 1.14 1.14 3.07 2.66 2.44 2.31 2.24 2.20 2.18 2.17 2.16 2.15 0.22 0.63 0.85 0.98 1.05 1.09 1.12 1.13 1.14 1.14 3.07 2.66 2.44 2.31 2.24 2.20 2.18 2.17 2.16 2.15 0.22 0.63 0.85 0.98 1.05 1.09 1.12 1.13 1.14 1.14 3.07 2.66 2.44 2.32 2.24 2.20 2.18 2.17 2.16 2.15 0.22 0.63 0.85 0.98 1.05 1.09 1.12 1.13 1.14 1.14 3.07 2.66 2.44 2.31 2.24 2.20 2.18 2.17 2.16 2.15 0.22 0.63 0.85 0.98 1.05 1.09 1.12 1.13 1.14 1.14 3.07 2.66 2.44 2.31 2.24 2.20 2.18 2.17 2.16 2.15 0.22 0.63 0.85 0.98 1.05 1.09 1.12 1.13 1.14 1.14 3.07 2.66 2.44 2.32 2.25 2.21 2.18 2.17 2.16 2.15 0.22 0.63 0.85 0.98 1.05 1.09 1.12 1.13 1.14 1.14 3.07 2.66 2.44 2.32 2.24 2.20 2.18 2.17 2.16 2.15 0.22 0.63 0.85 0.98 1.05 1.09 1.12 1.13 1.14 1.14 ];


tiempo = linspace(0,1,200);

figure(8)
hold on
plot(tiempo,R)
plot(tiempo,Y)
%plot(tiempo,U)
plot(tiempo,Usat)
title('Seguimiento de la referencia con Pole Placement')
xlabel('tiempo[segundos]') 
ylabel('Amplitud[volts]') 
legend('Referencia','Salida','Control','Control_{sat}')
