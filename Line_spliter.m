function [v] = Line_spliter(P1,P2,i)
    %Esta función genera un vector v conformado por los i puntos que conectan a
    %P1 con P2
    for t = 0:1/i:1
        a = round(i*t + 1);
        v(1,a) = P1(1)+t*(P2(1)-P1(1));
        v(2,a) = P1(2)+t*(P2(2)-P1(2));
    end
    disp(v)
end