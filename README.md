# Metodo-de-Pieper
Código para la resolución de la cinemática inversa de un manipulador tipo serial de 6 grados de libertad mediante el método de Pieper [Piper][Craig, pag. 114].
Como caso de ejemplo se toma el robot IRB120 de ABB en base al modelo presentado en [Gaudreault].

Instrucciones:
    - El archivo CI_IRB120.m sirve para analizar los elementos de la matriz de roto-traslación T de interes.
    
    - El archivo Ejemplo_Pieper.m sirve como ejemplo para el calculo de la cinematica inversa mediante el método de Pieper.

-----------
Referencias
----------- 

      [Gaudreault] Martin Gaudreault, Ahmed Joubair and Ilian Bonev,
                   "Self-Calibration of an Industrial Robot Using a Novel
                    Affordable 3D Measuring Device", Article SENSORS, 10
                    October 2018.

      [Craig] John J. Craig, "Introduccion a la robotica", 
              Pearson, 3nd Edicion, 2006.

      [Piper] Donald Lee Pieper, "The Kinematics of manipulators under 
              computer control", Stanford University, 1968. 
