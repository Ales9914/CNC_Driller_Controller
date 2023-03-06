function [Punto_A] = Superduper_point_acelerator(alpha,w,Punto_B,Dist_ba,Ang)
%Calculador de aceleracion de un punto con otro punto y aceleracion angular
    alpha_v = [0 0 alpha];
%     w_v = [0 0 w];
    Rba = [Dist_ba*cos(Ang) Dist_ba*sin(Ang) 0];
    Punto_A = Punto_B + cross(alpha_v,Rba) - w^2*Rba;
end

