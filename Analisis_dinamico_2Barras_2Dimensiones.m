%Análisis dinámico 2 barras - 2 dimensiones
clc
clear
F = 10;
vb = 1;
m1 = 1;
m2 = 1;
g = 9.81;
theta_1 = pi/4;
theta_2 = 5.114;
S1 = sin(theta_1);
S2 = sin(theta_2);
C2 = cos(theta_2);
C1 = cos(theta_1);
%Solución 2D Ay Ax By1 By2 Bx2 Cx Cy
A2 = [1 0 1 0 0 0 0 0;
     0 1 0 1 0 0 0 0;
     -C1 S1 C1 -S1 0 0 0 0;
     0 0 0 0 1 0 0 1;
     0 0 0 0 0 1 1 0;
     0 0 0 0 C2 -S2 S2 -C2
     0 0 0 1 0 1 0 0;
     0 0 1 0 1 0 0 0];
b2 = [0 0 0 0 0 0 0 F];
x2 = A2\b2';










