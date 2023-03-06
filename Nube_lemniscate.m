function [X_ocho,Y_ocho] = Nube_lemniscate(r,y_cent,x_cent,step_points,estech_fact)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
t_ocho = 0:step_points:6.3;
Y_ocho = y_cent + r.*sqrt(2).*cos(t_ocho).*sin(t_ocho)./(sin(t_ocho).^2 + 1);
X_ocho = x_cent + estech_fact*(r.*sqrt(2).*cos(t_ocho)./(sin(t_ocho).^2 + 1)); 
end
