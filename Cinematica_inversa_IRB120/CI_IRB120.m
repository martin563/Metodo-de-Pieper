%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Fecha: 30/06/2022
% Autor: Martin Oviedo
% Gmail: eliasoviedo1718@gmail.com
%
% Resumen :
%    Pruebas de cinematica inversa para el robot IRB120.
%
%       Seccion A) ==> Parametros del IRB120 segun [Gaudreault]
%
%       Seccion B) ==> Creo las matrices simbolicas de rototraslacion
%
%       Seccion C) ==> Extraigo elementos de las T a analizar
%
% -----------
% Referencias
% ----------- 
%
%       [Katayama] Tohru Katayama, "Subspace Methods for System Identification", 
%                  Springer, 1nd Edition. 
%
%       [Gaudreault] Martin Gaudreault, Ahmed Joubair and Ilian Bonev,
%                    "Self-Calibration of an Industrial Robot Using a Novel
%                     Affordable 3D Measuring Device", Article SENSORS, 10
%                     October 2018.
%                 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc, clear all, close all

%% Constantes

mm = 10^(-3);
ang_a_rad = pi/180;
rad_a_ang = 1/ang_a_rad;


%% A) Parametros del IRB120 segun [Gaudreault]

%Valores numericos de los parametros para la tabla DH
a_num = [0;0;270*mm;70*mm;0;0];
d_num = [290*mm;0;0;302*mm;0;0;72*mm];
alfa_num = [0;-pi/2;0;-pi/2;pi/2;-pi/2];

v = [a_num; d_num; alfa_num];

%% B) Creo las matrices simbolicas de rototraslacion

%Declaro parametros y variables simbolicas para la tabla DH
syms c1 c2 c3 c4 c5 c6 s1 s2 s3 s4 s5 s6 real;
syms a1 a2 a3 a4 a5 a6 real;
syms d1 d2 d3 d4 d5 d6 d7 real;
syms f1 f2 f3 f4 f5 f6 real;

a = [a1;a2;a3;a4;a5;a6];
d = [d1;d2;d3;d4;d5;d6;d7];
alfa = [f1;f2;f3;f4;f5;f6];
param = [a;d;alfa];

%Matrices simbolicas resultados de la tabla de DH
T0r_1 = sym('[1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, d1; 0, 0, 0, 1]');
T1r0 = sym('[c1, -s1, 0, a1; s1*cos(f1), c1*cos(f1), -sin(f1), 0; s1*sin(f1), c1*sin(f1), cos(f1), 0; 0, 0, 0, 1]');
T2r1 = sym('[c2, -s2, 0, a2; s2*cos(f2), c2*cos(f2), -sin(f2), -sin(f2)*d2; s2*sin(f2), c2*sin(f2), cos(f2), cos(f2)*d2; 0, 0, 0, 1]');
T3r2 = sym('[c3, -s3, 0, a3; s3*cos(f3), c3*cos(f3), -sin(f3), -sin(f3)*d3; s3*sin(f3), c3*sin(f3), cos(f3), cos(f3)*d3; 0, 0, 0, 1]');
T4r3 = sym('[c4, -s4, 0, a4; s4*cos(f4), c4*cos(f4), -sin(f4), -sin(f4)*d4; s4*sin(f4), c4*sin(f4), cos(f4), cos(f4)*d4; 0, 0, 0, 1]');
T5r4 = sym('[c5, -s5, 0, a5; s5*cos(f5), c5*cos(f5), -sin(f5), -sin(f5)*d5; s5*sin(f5), c5*sin(f5), cos(f5), cos(f5)*d5; 0, 0, 0, 1]');
T6r5 = sym('[c6, -s6, 0, a6; s6*cos(f6), c6*cos(f6), -sin(f6), 0; s6*sin(f6), c6*sin(f6), cos(f6), 0; 0, 0, 0, 1]');
T7r6 = sym('[1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, d7; 0, 0, 0, 1]');

% Utilizó el cambio de base para obtener la Matriz T {H} ==> {B}
T2r0 = T0r_1*T1r0*T2r1;
T3r0 = T2r0*T3r2;
T4r0 = T3r0*T4r3;
T5r0 = T4r0*T5r4;
T6r0 = T5r0*T6r5;
T7r0 = T6r0*T7r6;

%% C) Extraigo elementos de las T a analizar

T = T4r3*T5r4*T6r5;

x(param) = T(1,4);
y(param) = T(2,4);
z(param) = T(3,4);
r11(param) = T(1,1);
r12(param) = T(1,2);
r13(param) = T(1,3);
r21(param) = T(2,1);
r22(param) = T(2,2);
r23(param) = T(2,3);
r31(param) = T(3,1);
r32(param) = T(3,2);
r33(param) = T(3,3);

x_num = simplify(subs(x, param, v));
y_num = simplify(subs(y, param, v));
z_num = simplify(subs(z, param, v));
r11_num = simplify(subs(r11, param, v));
r12_num = simplify(subs(r12, param, v));
r13_num = simplify(subs(r13, param, v));
r21_num = simplify(subs(r21, param, v));
r22_num = simplify(subs(r22, param, v));
r23_num = simplify(subs(r23, param, v));
r31_num = simplify(subs(r31, param, v));
r32_num = simplify(subs(r32, param, v));
r33_num = simplify(subs(r33, param, v));
