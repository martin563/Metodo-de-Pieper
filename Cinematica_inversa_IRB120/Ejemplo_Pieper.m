%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Fecha: 09/07/2022
% Autor: Martin Oviedo
% Gmail: eliasoviedo1718@gmail.com
%
% Resumen :
%    Pruebas de cinematica inversa para el robot IRB120 con el metodo de 
%    Piper [Craig, pag.114].
%
%       Seccion A) ==> Defino parametros y creo la matriz simbolica de 
%                      cinematica directa 
%
%       Seccion B) ==> Defino angulos de prueba e implemento el metodo de 
%                      Piper 
%
%       Seccion C) ==> Verifico resultados
%
% -----------
% Referencias
% ----------- 
%
%       [Gaudreault] Martin Gaudreault, Ahmed Joubair and Ilian Bonev,
%                    "Self-Calibration of an Industrial Robot Using a Novel
%                     Affordable 3D Measuring Device", Article SENSORS, 10
%                     October 2018.
%
%       [Craig] John J. Craig, "Introduccion a la robotica", 
%               Pearson, 3nd Edicion, 2006.
%
%       [Piper] Donald Lee Pieper, "The Kinematics of manipulators under 
%               computer control", Stanford University, 1968. 
%                 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc, clear all, close all

%% Constantes

mm = 10^(-3);
ang_a_rad = pi/180;
rad_a_ang = 1/ang_a_rad;

%% A) Defino parametros y creo la matriz simbolica de cinematica directa

%Valores numericos de los parametros para la tabla DH
a = [0;0;270;70;0;0]*mm;
d = [290;0;0;302;0;72]*mm;
alfa = [0;-90;0;-90;90;-90]*ang_a_rad;

syms q1 q2 q3 q4 q5 q6 real

%Tabla de Denavit-Hartenberg modificada del IRB120
p0.d=d(1)    ; p0.a=0       ; p0.alfa=0       ;p0.q=0;
p1.d=0       ; p1.a=a(1)    ; p1.alfa=alfa(1) ;p1.q=q1;
p2.d=d(2)    ; p2.a=a(2)    ; p2.alfa=alfa(2) ;p2.q=q2;
p3.d=d(3)    ; p3.a=a(3)    ; p3.alfa=alfa(3) ;p3.q=q3;
p4.d=d(4)    ; p4.a=a(4)    ; p4.alfa=alfa(4) ;p4.q=q4;
p5.d=d(5)    ; p5.a=a(5)    ; p5.alfa=alfa(5) ;p5.q=q5;
p6.d=0       ; p6.a=a(6)    ; p6.alfa=alfa(6) ;p6.q=q6;
p7.d=d(6)    ; p7.a=0       ; p7.alfa=0       ;p7.q=0;

% La función transf_DHcraig calcula T {i} ==> {i-1} para cada fila de la tabla
T0r_1 =  transf_DHcraig(p0);
T1r0 = transf_DHcraig(p1);
T2r1 = transf_DHcraig(p2);
T3r2 = transf_DHcraig(p3);
T4r3 = transf_DHcraig(p4);
T5r4 = transf_DHcraig(p5);
T6r5 = transf_DHcraig(p6);
T7r6 = transf_DHcraig(p7);

% Utilizó el cambio de base para obtener la Matriz T {H} ==> {B}
T6r0 = simplify(T0r_1*T1r0*T2r1*T3r2*T4r3*T5r4*T6r5*T7r6);

%% B) Defino angulos de prueba e implemento el metodo de Piper

%Creo los angulos de prueba
angulos_de_prueba = [pi/6,pi/3,pi/4,pi/3,pi/4,pi];
q_num = angulos_de_prueba*rad_a_ang;

%Parametros que se pasan a metodo_de_piper
DH = [alfa, a, d];

%Matriz de cinematica directa numerica - Caso de prueba
TH_B = double(subs(T6r0,{q1,q2,q3,q4,q5,q6},{angulos_de_prueba})); 

%Para el IRB120 se aplica la siguiente cuenta para que se cumpla las
%condiciones del metodo de Pieper
T_dato = T0r_1\TH_B;
T_dato = T_dato/T7r6;

%Conjunto de posibles soluciones (en su orden correspondiente)
[conj, N_sol] = metodo_de_pieper(DH,T_dato); 

%% C) Verifico resultados

%Resultados de la cinematica inversa con las posibles soluciones (en su orden correspondiente)
sol = zeros(4,4,N_sol);
rev = sol;

for i=1:N_sol
   sol(:,:,i) = single(subs(T6r0,{q1,q2,q3,q4,q5,q6},{conj(i,:)} ));
   rev(:,:,i) = TH_B - sol(:,:,i);
end

%Reviso si cumplen la cinematica directa
for i=1:N_sol
   rev(:,:,i) 
end
