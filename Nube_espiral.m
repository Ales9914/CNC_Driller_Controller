function [X_espiral,Y_espiral] = Nube_espiral(a,b,y_cent,x_cent,step_points,estrech_fact)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
t_espiral = 0:step_points:15;
Y_espiral = y_cent + estrech_fact*(a + a.*t_espiral).*sin(t_espiral);
X_espiral = x_cent + estrech_fact*(b + b.*t_espiral).*cos(t_espiral);
end

