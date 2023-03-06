%Análisis dinámico 3D completo
function [vect,vect_b] = Analisis_dinamico_3D_completo(results,vb)
    g = 9.81;
    F = 1000;
    h_pas = 0.015;
    tool_m = 0.3;
    altura = 0.2;
    M = altura*F;
    m1 = 0.251;
    m2 = 0.251;
    m3 = 1.543;
    m4 = 0.251;
    mb = 0;
    I1 = m1*vb^2/12;
    I2 = m2*vb^2/12;
    I3 = m3*vb^2/12;
    I4 = m4*vb^2/12;

    % a1 = results(4:5,1)';
    % a2 = results(4:5,2)';
    % a3 = results(4:5,3)';
    % a4 = results(4:5,4)';
    % ab = results(4:5,5)';
    a1 = [0 0];
    a2 = [0 0];
    a3 = [0 0];
    a4 = [0 0];
    ab = [0 0];

    alpha_1 = 0;
    alpha_2 = 0;
    alpha_3 = 0;
    alpha_4 = 0;
    % alpha_1 = results(3,1)';
    % alpha_2 = results(3,2)';
    % alpha_3 = results(3,3)';
    % alpha_4 = results(3,4)';

    theta_1 = results(1,1)';
    theta_2 = results(1,2)';
    theta_3 = results(1,3)';
    theta_4 = results(1,4)';
    S1 = sin(theta_1);
    S2 = sin(theta_2);
    S3 = sin(theta_3);
    S4 = sin(theta_4);
    C1 = cos(theta_1);
    C2 = cos(theta_2);
    C3 = cos(theta_3);
    C4 = cos(theta_4);



    %Sistema matrix
    %Ay By1 Ax Bx1 By2 Cy Cx Bx2 Dx Bx3 By3 Dy Oy Ox
    A = [1     1     0     0     0     0     0     0     0     0     0     0     0     0;
         0     0     1     1     0     0     0     0     0     0     0     0     0     0;
         0     0     0     0     1     1     0     0     0     0     0     0     0     0;
         0     0     0     0     0     0     1     1     0     0     0     0     0     0;
         0     0     0     0     0     0     0     0     1     1     0     0     0     0;
         0     0     0     0     0     0     0     0     0     0     1     1     0     0;
         0     0     0     0     0     0     0     0     0     0     0    -1     1     0;
         0     0     0     0     0     0     0     0    -1     0     0     0     0     1;
         0     0     0     1     0     0     0     1     0     1     0     0     0     0;
         0     1     0     0     1     0     0     0     0     0     1     0     0     0;
       -C1    C1    S1    -S1    0     0     0     0     0     0     0     0     0     0;
         0     0     0     0    C2    -C2   S2    -S2    0     0     0     0     0     0;
         0     0     0     0     0     0     0     0    S3   -S3    C3   -C3     0     0;
         0     0     0     0     0     0     0     0    C4     0     0   -S4    -S4    C4];


     b = [m1*a1(2);
          m1*a1(1);
          m2*a2(2);
          m2*a2(1); 
          m3*a3(1); 
          m3*a3(2); 
          m4*a4(2);
          m4*a4(1); 
          mb*ab(1); 
          mb*ab(2)-F;
          2*I1*alpha_1/vb;
          2*I2*alpha_2/vb;
          2*I3*alpha_3/vb;
          2*I4*alpha_4/vb];

    x = A\b;
    
    %Ecuaciones off-matrix
    Az = m1*g;
    May = -0.5*m1*g*vb*C1 -h_pas*x(4);
    Max = 0.5*m1*g*vb*C1 +h_pas*x(2);
    Cz = m2*g;
    Mcy = m2*g*vb*sin(theta_2-pi/2)/2 -h_pas*x(8);
    Mcx = m2*g*vb*cos(theta_2-pi/2)/2 +h_pas*x(5);
    Dz = m3*g;
    Mdy = m3*g*vb*sin(theta_3-pi/2)/2 - h_pas*x(10);
    Mdx = m3*g*vb*cos(theta_3-pi/2)/2 - M +h_pas*x(11);
    Oz = m4*g + m3*g + tool_m*g;
    Moy = g*vb*(m4*C4 + m3*sin(theta_3-90))/2 +h_pas*x(9);
    Mox = g*vb*(m4*S4 + m3*cos(theta_3-90))/2 -h_pas*x(12);
    
    
    %Soluciones off-matrix
    %Ay By1 Ax Bx1 By2 Cy Cx Bx2 Dx Bx3 By3 Dy Oy Ox

    vect = [x(3) x(7) x(9) x(14); 
            x(1) x(6) x(12) x(13);
            Az Cz Dz Oz; 
            Max Mcx Mdx Mox; 
            May Mcy Mdy Moy];
    vect_b = [x(4) x(8) x(10);
              x(2) x(5) x(11)];
end






