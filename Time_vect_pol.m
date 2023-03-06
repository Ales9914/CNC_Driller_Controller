function [T_vect, T_vect_der, T_vect_der_2] = Time_vect_pol(n,t)
    T_vect = [];
    for i = 1:1:n+1
        T_vect(i) = t^(n+1-i);
    end
    for i = 1:1:n
        T_vect_der(i) = (n+1-i)*t^(n-i);
    end
    T_vect_der(n+1) = 0;
    for i = 1:1:n-1
        T_vect_der_2(i) = (n+1-i)*(n-i)*t^(n-i-1);
    end
    T_vect_der_2(n) = 0;
    T_vect_der_2(n+1) = 0;
end

