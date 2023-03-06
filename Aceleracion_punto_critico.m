function [Ace_C1,Ace_C2,Ace_C3,Ace_C4,Ace_B] = Aceleracion_punto_critico(Punto_critico,ace_arr,vel_arr,pos_arr,vb1)
%Paso 14: Cálcular las aceleraciones en los centroides
    Sln_Ace_Punto_crit = ace_arr(:,Punto_critico);%theta1, theta2, theta3, theta4, T1, T2
    Sln_Vel_Punto_crit = vel_arr(:,Punto_critico);
    Sln_Pos_Punto_crit = pos_arr(:,Punto_critico);
    Ace_O = [0 0 0];
    Ace_A = [0 Sln_Ace_Punto_crit(5) 0];
    Ace_C = [0 Sln_Ace_Punto_crit(6) 0];
    [Ace_B] = Superduper_point_acelerator(Sln_Ace_Punto_crit(1),Sln_Vel_Punto_crit(1),Ace_A,vb1,Sln_Pos_Punto_crit(1));
    [Ace_D] = Superduper_point_acelerator(Sln_Ace_Punto_crit(4),Sln_Vel_Punto_crit(4),Ace_O,vb1,Sln_Pos_Punto_crit(4));
    [Ace_C1] = Superduper_point_acelerator(Sln_Ace_Punto_crit(1),Sln_Vel_Punto_crit(1),Ace_A,vb1/2,Sln_Pos_Punto_crit(1));
    [Ace_C2] = Superduper_point_acelerator(Sln_Ace_Punto_crit(2),Sln_Vel_Punto_crit(2),Ace_C,vb1/2,Sln_Pos_Punto_crit(2));
    [Ace_C3] = Superduper_point_acelerator(Sln_Ace_Punto_crit(3),Sln_Vel_Punto_crit(3),Ace_D,vb1/2,Sln_Pos_Punto_crit(3));
    [Ace_C4] = Superduper_point_acelerator(Sln_Ace_Punto_crit(4),Sln_Vel_Punto_crit(4),Ace_O,vb1/2,Sln_Pos_Punto_crit(4));
end

