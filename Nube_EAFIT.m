function [X,Y] = Nube_EAFIT(y_cent,x_cent,step_points,estech_fact)
    points = [0.5 0  0 0.35 0 0.5 0.75 0.875 0.625 1  1  1.35 1 2 1.75 1.75 1.5 2.35 2.35 2.1 2.6;
              1   1 0.5 0.5  0  0   1    0.5   0.5  0 0.5 0.5  1 1  1     0   0    0   1    1   1];
    topo = [1 2 3 4 3 5 6 7 8 9 8 10 11 12 11 13 14 15 16 17 18 19 20;
            2 3 4 3 5 6 7 8 9 8 10 11 12 11 13 14 15 16 17 18 19 20 21];
    [~,size_topo] = size(topo);
    X = [];
    Y = [];
    for i = 1:1:size_topo
        v = Line_spliter(points(:,topo(1,i)),points(:,topo(2,i)),step_points);
        X = [X v(1,:)];
        Y = [Y v(2,:)];
    end
    X = x_cent  + estech_fact.*(X- 1.25);
    Y = y_cent + estech_fact.*Y;
end