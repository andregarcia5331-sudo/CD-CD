% =========================================================================
% PRÁCTICA 6: Modelado de sistemas Eléctricos; convertidor CD-CD
% Ejercicio 1: Integración mediante ODE45 en MATLAB
% =========================================================================

clear; clc; close all;

%% 1. Definición de Parámetros del Circuito y Control
L = 2e-3;      % Inductancia: 2 mH
R = 10;        % Resistencia: 10 Ohms
C = 10e-6;     % Capacitancia: 10 uF
Uin = 32;      % Voltaje de entrada: 32 V

f_pwm = 100e3; % Frecuencia del PWM: 100 kHz
D = 0.40;      % Ciclo de trabajo (Duty Cycle): 40%
T_pwm = 1/f_pwm; % Periodo del PWM

%% 2. Condición de Simulación
tspan = [0, 0.005]; % Tiempo de simulación (5 ms)
x0 = [0; 0];         % Condiciones iniciales: iL(0) = 0 A, Vc(0) = 0 V

%% 3. Integración Numérica con ode45
% Se utiliza un solver con tamaño de paso máximo adaptado a la frecuencia de conmutación
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8, 'MaxStep', T_pwm / 50);
[t, x] = ode45(@(t, x) sistema_buck(t, x, L, R, C, Uin, D, T_pwm), tspan, x0, options);

% Extracción de los estados
iL = x(:, 1); % Corriente en el inductor
Vc = x(:, 2); % Voltaje en el capacitor

%% 4. Gráfica de Resultados
figure('Name', 'Simulación del Convertidor CD-CD (ODE45)', 'Color', 'w');

% Gráfica de la Corriente del Inductor
subplot(2,1,1);
plot(t * 1e3, iL, 'b', 'LineWidth', 1.5);
grid on;
title('Corriente en el Inductor $i_L(t)$', 'Interpreter', 'latex');
xlabel('Tiempo (ms)');
ylabel('Corriente (A)');

% Gráfica del Voltaje del Capacitor
subplot(2,1,2);
plot(t * 1e3, Vc, 'r', 'LineWidth', 1.5);
grid on;
title('Voltaje en el Capacitor $V_c(t)$', 'Interpreter', 'latex');
xlabel('Tiempo (ms)');
ylabel('Voltaje (V)');

%% 5. Función de las Ecuaciones de Estado (Modelo Dinámico)
function dxdt = sistema_buck(t, x, L, R, C, Uin, D, T_pwm)
    % Asignación de variables de estado
    % x(1) = iL(t)
    % x(2) = Vc(t)
    
    % Generación de la señal PWM d(t)
    t_mod = mod(t, T_pwm);
    if t_mod < D * T_pwm
        d = 1; % Mosfet ON
    else
        d = 0; % Mosfet OFF
    end
    
    % Sistema de ecuaciones diferenciales
    diL_dt = -(1/L)*x(2) + (Uin/L)*d;
    dVc_dt =  (1/C)*x(1) - (1/(R*C))*x(2);
    
    dxdt = [diL_dt; dVc_dt];
end