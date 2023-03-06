clc
clear
close all
L = 0.2476;
vb1 = 0.255;
vb2 = 0.255;
A = (vb1^2-vb2^2+L^2)/2;
Pul_per_rev = 400;
Speed_factor = 5;



%Paso 0: Llamar la nube de puntos
%Nota: Minimo Y posible es sqrt(vb1^2-(L/2)^2)
%Nota: Minimo X posible es L-vb2 y máximo vb1

% [x,y] = Nube_circulo(5*L/10,L,L/2,0.25);
[x,y] = Nube_lemniscate(5*L/10,14*L/10,L/2,0.1,0.5); %(r,y_cent,x_cent,step_points,estech_fact)
% [x,y] = Nube_espiral(L/40,L/40,13*L/10,L/2,0.1,1); %(a,b,y_cent,x_cent,step_points,estrech_fact)
% [x,y] = Nube_cool_shape(1,0,9,13*L/10,L/2,0.05,0.07);
% [x,y] = Nube_EAFIT(14*L/10,L/2,10,0.075);%(y_cent,x_cent,step_points,estech_fact)

%Paso 1: Determinar el movimiento del primer tornillo para lograr la
%coordenada en Y
[~,size_x] = size(x);
for i = 1:size_x
    theta_1 = acos(x(i)/vb1);
    vp1(i) = y(i)-vb1.*sin(theta_1);
    if vp1(i) < 0
        y(i)
        theta_1
        vb1.*sin(theta_1)
        pause
    end
end

%Paso 2: Determinar el vector de d_h necesarios para lograr las coordenadas
%en X

d_h = sqrt(vb1.^2-x.^2)-sqrt(vb1.^2-L^2+2.*L.*x-x.^2)
% plot(x,y)

%Paso 3: Determinar el movimiento del segundo tornillo.
vp2 = vp1 + d_h;
[size_pr_vp2] = size(find(vp2<0));
if size_pr_vp2 ~= 0
    disp("Second scroll lower than 0")
end
%3.5 Activar si el tornillo comienza desde 0 o desde la posición incial
% vp1 = [0 vp1];
% vp2 = [0 vp2];
% size_x = size_x + 1

%Paso 4: Determinar cuanto debe desplazarse el tornillo desde su última
%posición
p_inicial_prim_tor = 0;
p_inicial_seg_tor = 0;
Del_prim_tor = [vp1(1)-p_inicial_prim_tor];
Del_segu_tor = [vp2(1)-p_inicial_prim_tor];
for i=2:size_x
    Del_prim_tor(i) = vp1(i)-vp1(i-1);
    Del_segu_tor(i) = vp2(i)-vp2(i-1);
end

%Paso 5: Determinar cuantas vueltas debe dar el motor para lograr ese
%avance: 4mm/vuelta

rev_pri_mot = Del_prim_tor./4;
rev_seg_mot = Del_segu_tor./4;

%Paso 6: Pulsos necesarios para lograr esa cantidad de vueltas:

Pul_pri_mot = rev_pri_mot*Pul_per_rev;
Pul_seg_mot = rev_seg_mot*Pul_per_rev;

%Paso 7: Redondear los pulsos a números enteros

Pul_pri_mot_r = round(Pul_pri_mot);
Pul_seg_mot_r = round(Pul_seg_mot);

%Paso 8: Calcular el delay para que ambos tornillos lleguen al mismo tiempo
%al punto.

%Paso 9: Calcular el vector de dirección
vect_dir_pri_mot = [];
for i = 1:size_x
    [~,size_vect_dir] = size(vect_dir_pri_mot);
    if Pul_pri_mot_r(i)<0
        vect_dir_pri_mot(size_vect_dir+1) = 2;
    elseif Pul_pri_mot_r(i)>0
        vect_dir_pri_mot(size_vect_dir+1) = 1;
    elseif Pul_pri_mot_r(i) == 0
        vect_dir_pri_mot(size_vect_dir+1) = 0;
    end
end


%Paso 10: plot las funciones de movimiento del mecanismo
% plot(x,y)
% grid on
% figure
% for i = 1:1:size_x
%     node_i = [0 vp1(i); x(i) y(i); L vp2(i)];
%     plot(node_i(:,1),node_i(:,2),'b')
%     grid on
%     hold on
%     pause(0.1)
% end

% Paso 11: Interpolar los movimientos del tornillo para hallar una función de tiempo
step = 1:1:size_x;
step = step/Speed_factor;
%Coeficientes del polinomio
n1 = 9;
n2 = 13;
T1_ft = polyfit(step',vp1',n1);
T2_ft = polyfit(step',vp2',n2);

%Calcular el error del polinomio
values_T1_ft = polyval(T1_ft,step');
for i = 1:1:size_x
    vect_err(i) = abs(values_T1_ft(i)-vp1(i))/abs(vp1(i));
end
values_T2_ft = polyval(T2_ft,step');
for i = 1:1:size_x
    vect_err(i) = abs(values_T2_ft(i)-vp2(i))/abs(vp2(i));
end
% 
% 
figure
plot(step',vp1','k')
grid on
hold on
plot(step',values_T1_ft','r')
legend('Función ideal','Función obtenida')
ylabel('P1 [m]')
xlabel('t [s]')

figure
plot(step',vp2','k')
grid on
hold on
plot(step',values_T2_ft','r')
legend('Función ideal','Función obtenida')
ylabel('P2 [m]')
xlabel('t [s]')

%Paso 11.2 No usar un polinomio
% x0 = [0.05 1 2 2 0.24];
% A1 = 0.07;
% A2 = 0.025;
% [coef_1] = fit_the_model(step,vp1,x0,A1,A2);
% values_T1_ft = A1*sin(coef_1(1)*step + coef_1(2))+A2*sin(coef_1(3)*step + coef_1(4))+coef_1(5);
% 
% x0 = [0.01 1 2 2 0.24];
% A3 = 0.07;
% A4 = 0.025;
% [coef_2] = fit_the_model(step,vp2,x0,A1,A2);
% values_T2_ft = A3*sin(coef_2(1)*step + coef_2(2))+A4*sin(coef_2(3)*step + coef_2(4))+coef_2(5);

% figure
% plot(step',vp1','k')
% grid on
% hold on
% plot(step',values_T1_ft','r')
% legend('Función ideal','Función obtenida')
% ylabel('P1 [m]')
% xlabel('t [s]')
% 
% figure
% plot(step',vp2','k')
% grid on
% hold on
% plot(step',values_T2_ft','r')
% legend('Función ideal','Función obtenida')
% ylabel('P2 [m]')
% xlabel('t [s]')

%Paso 12: Función de aceleracion y posicion para la función interpolada
% q = [0.5 1.58 pi/2 0 vp1(1) vp2(1)]';%theta1, theta2, theta3, theta4, T1, T2
q = [0.38 1.815 pi/2 0 vp1(1) vp2(1)]';
vel_arr = [];
ace_arr = [];
pos_arr = [];
pos_b_arr = [];
figure
for i = step
    t = i;
    tol = 1;
    iter = 1;
    theta_t1 = pi/2;
    theta_L = 0;
    [T2_vect, T2_vect_der, T2_vect_der2] = Time_vect_pol(n2,t);
    [T1_vect, T1_vect_der, T1_vect_der2] = Time_vect_pol(n1,t);
    T1_eval_on_t = dot(T1_vect,T1_ft);
    T2_eval_on_t = dot(T2_vect,T2_ft);
%     T1_eval_on_t = A1*sin(coef_1(1)*t + coef_1(2))+A2*sin(coef_1(3)*t + coef_1(4))+coef_1(5);
%     T2_eval_on_t = A3*sin(coef_2(1)*t + coef_2(2))+A4*sin(coef_2(3)*t + coef_2(4))+coef_2(5);
    while and(iter<100,tol>1e-6)
        phi = [vb1*cos(q(4))+vb1*cos(q(3))-vb1*cos(q(2))-q(6)*cos(theta_t1)-L*cos(theta_L);
               vb1*sin(q(4))+vb1*sin(q(3))-vb1*sin(q(2))-q(6)*sin(theta_t1)-L*sin(theta_L);
               q(6)-T2_eval_on_t;
               q(5)*cos(theta_t1)+vb1*cos(q(1))-vb1*cos(q(3))-vb1*cos(q(4));
               q(5)*sin(theta_t1)+vb1*sin(q(1))-vb1*sin(q(3))-vb1*sin(q(4));
               q(5)-T1_eval_on_t];
        J = [      0                vb1*sin(q(2))  -vb1*sin(q(3))    -vb1*sin(q(4))         0      -cos(theta_t1);
                   0               -vb1*cos(q(2))   vb1*cos(q(3))     vb1*cos(q(4))         0      -sin(theta_t1);
                   0                     0                0                 0               0             1;
             -vb1*sin(q(1))              0          vb1*sin(q(3))     vb1*sin(q(4))    cos(theta_1)       0;
              vb1*cos(q(1))              0         -vb1*cos(q(3))    -vb1*cos(q(4))    sin(theta_1)       0;
                   0                     0                0                 0                1            0];
        q_1 = -(J\phi) + q;
        tol = norm(phi);
        iter = iter+1;
        q = q_1;
    end
    pos_arr = [pos_arr q];
    %Generate nodes x y
    node = [0 0; 0 T1_eval_on_t; vb1*cos(q(1)) T1_eval_on_t+vb1*sin(q(1)); ...
         vb1*cos(q(4)) vb1*sin(q(4)); 0 0; L 0; L T2_eval_on_t;L + vb1*cos(q(2)) T2_eval_on_t+vb1*sin(q(2))];
%     pos_b_arr = [pos_b_arr; vb1*cos(q(1)) T1_eval_on_t+vb1*sin(q(1))];
%     plot(x,y, 'k')
%     hold on
    plot(node(:,1), node(:,2), 'b')
%     axis([0 20*L/10 0 20*L/10])
    grid on
    hold on
    pause(1/Speed_factor)
%     
%     Velocidad ángular
    
    T1_deriv_eval_on_t = dot(T1_vect_der,T1_ft);
    T2_deriv_eval_on_t = dot(T2_vect_der,T2_ft);
%     T1_deriv_eval_on_t = A1*coef_1(1)*cos(coef_1(1)*t+coef_1(2)) + A2*coef_1(3)*cos(coef_1(3)*t+coef_1(4));
%     T2_deriv_eval_on_t = A3*coef_2(1)*cos(coef_2(1)*t + coef_2(2))+A4*coef_2(3)*cos(coef_2(3)*t+coef_2(4));
    phi_t = [0 0 -T2_deriv_eval_on_t 0 0 -T1_deriv_eval_on_t]';
    q_vel = -inv(J)*phi_t;
    vel_arr = [vel_arr [q_vel;i]];
    
    %Aceleración ángular
    J_dt = [            0                vb1*cos(q(2))*q_vel(2)  -vb1*cos(q(3))*q_vel(3)    -vb1*cos(q(4))*q_vel(4)       0      0;
                        0                vb1*sin(q(2))*q_vel(2)  -vb1*sin(q(3))*q_vel(3)    -vb1*sin(q(4))*q_vel(4)       0      0;
                        0                         0                        0                             0                0      0;
             -vb1*cos(q(1))*q_vel(1)              0               vb1*cos(q(3))*q_vel(3)     vb1*cos(q(3))*q_vel(4)       0      0;
             -vb1*sin(q(1))*q_vel(1)              0               vb1*sin(q(3))*q_vel(3)     vb1*sin(q(3))*q_vel(4)       0      0;
                        0                         0                        0                             0                0      0];
    T1_deriv2_eval_on_t = dot(T1_vect_der2,T1_ft);
    T2_deriv2_eval_on_t = dot(T2_vect_der2,T2_ft);
%     T1_deriv2_eval_on_t = -A1*coef_1(1)*coef_1(1)*sin(coef_1(1)*t+coef_1(2)) - A2*coef_1(3)*coef_1(3)*sin(coef_1(3)*t+coef_1(4));
%     T2_deriv2_eval_on_t = -A3*coef_2(1)*coef_2(1)*sin(coef_2(1)*t + coef_2(2)) - A4*coef_2(3)*coef_2(3)*sin(coef_2(3)*t+coef_2(4));
    phi_tt = [0 0 -T2_deriv2_eval_on_t 0 0 -T1_deriv2_eval_on_t]';
    q_ace = inv(J)*(-phi_tt-J_dt*q_vel);
%     ace_arr = [ace_arr [q_ace;i]];
    ace_arr = [ace_arr q_ace];
end

plot(x,y, 'k')
hold on
plot(pos_b_arr(:,1), pos_b_arr(:,2), 'r')
grid on
legend('Trayectoria ideal','Trayectoria obtenida')
ylabel('P1 [m]')
xlabel('t [s]')

%Paso 13: Que posición tiene más aceleración
[max_ace_arr,norm_ace_arr] = Paso_13(x,y,vb1,L,ace_arr,pos_arr,size_x,step,T1_ft,T2_ft);

%Paso 14: Cáclular la aceleración de todos los nodos y barras en los puntos
%criticos
[Ace_C1_64,Ace_C2_64,Ace_C3_64,Ace_C4_64,Ace_B_64] = Aceleracion_punto_critico(64,ace_arr,vel_arr,pos_arr,vb1);
[Ace_C1_34,Ace_C2_34,Ace_C3_34,Ace_C4_34,Ace_B_34] = Aceleracion_punto_critico(34,ace_arr,vel_arr,pos_arr,vb1);

%Paso 15: Important shit results
results_64 = [[pos_arr(1:4,64)]' 0;[vel_arr(1:4,64)]' 0;[ace_arr(1:4,64)]' 0;
              [Ace_C1_64(1:2)' Ace_C2_64(1:2)' Ace_C3_64(1:2)' Ace_C4_64(1:2)' Ace_B_64(1:2)']]
results_34 = [[pos_arr(1:4,34)]' 0;[vel_arr(1:4,34)]' 0;[ace_arr(1:4,34)]' 0;
              [Ace_C1_34(1:2)' Ace_C2_34(1:2)' Ace_C3_34(1:2)' Ace_C4_34(1:2)' Ace_B_34(1:2)']]
          
%Paso 16: Results for the scrolls
Ts_arr = [pos_arr(5,:);
          vel_arr(5,:);
          ace_arr(5,:);
          pos_arr(6,:);
          vel_arr(6,:);
          ace_arr(6,:)];
      
      
%Paso 17: Análisis dinámico
[vect_dina_64,vect_dina_b_64] = Analisis_dinamico_3D_completo(results_64,vb1)
[vect_dina_32,vect_dina_b_32] = Analisis_dinamico_3D_completo(results_34,vb1)

% [Pul_pri_mot_r] = Separador_Comas_Arduino(vect_dir_pri_mot,Pul_pri_mot_r)





