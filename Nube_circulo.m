function [X_circulo,Y_circulo] = Nube_circulo(r,y_cent,x_cent,step_points)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
X_circulo = x_cent-r:step_points:x_cent+r;
Y_circulo_pos = sqrt(r^2-(X_circulo-x_cent).^2)+y_cent;
Y_circulo_neg = -sqrt(r^2-(X_circulo-x_cent).^2)+y_cent;
Y_circulo = [Y_circulo_pos flip(Y_circulo_neg)];
X_circulo = [X_circulo flip(X_circulo)];
end

