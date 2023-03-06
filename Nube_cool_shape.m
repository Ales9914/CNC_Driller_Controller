function [X_espiral,Y_espiral] = Nube_cool_shape(a,b,c,y_cent,x_cent,step_points,estrech_fact)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
t = 0:step_points:6.3;
Y_espiral = y_cent + estrech_fact*(cos(a*t) + cos(b*t)/2 + sin(c*t)/3);
X_espiral = x_cent + estrech_fact*(sin(a*t) + sin(b*t)/2 + cos(c*t)/3);
end
