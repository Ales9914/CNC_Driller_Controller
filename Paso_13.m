function [max_ace_arr,norm_ace_arr] = Paso_13(x,y,vb1,L,ace_arr,pos_arr,size_x,step,T1_ft,T2_ft)
    %Paso 13: Máxima aceleración
    %Aquí se calculan los vectores de aceleración según los criterios de
    %máxima aceleración o norma de aceleraciones para determinar que puntos
    %son criticos

    max_ace_arr = max(ace_arr(1:4,:));
    norm_ace_arr = [];
    for i = 1:1:size_x
        norm_ace_arr = [norm_ace_arr norm(ace_arr(1:4,i))];
    end
    figure
    plot(step,max_ace_arr)
    title('max_{alpha}')
    grid on
    ylabel('alpha [m/s^2]')
    xlabel('t [s]')
    
    figure
    plot(step,norm_ace_arr)
    title('norm_{alpha}')
    ylabel('alpha [m/s^2]')
    xlabel('t [s]')
    grid on


%     pos_crit = [1 23 34 43 57 64];
%     t = 64;
%     T2_vect = [t.^13 t.^12 t.^11 t.^10 t.^9 t.^8 t.^7 t.^6 t.^5 t.^4 t.^3 t.^2 t.^1 t.^0];
%     T1_vect = [t.^9 t.^8 t.^7 t.^6 t.^5 t.^4 t.^3 t.^2 t.^1 t.^0];
%     T1_eval_on_t = dot(T1_vect,T1_ft);
%     T2_eval_on_t = dot(T2_vect,T2_ft);
%     q = pos_arr(:,t);
%     node = [0 0; 0 T1_eval_on_t; vb1*cos(q(1)) T1_eval_on_t+vb1*sin(q(1)); ...
%             vb1*cos(q(4)) vb1*sin(q(4)); 0 0; L 0; L T2_eval_on_t;L + vb1*cos(q(2)) T2_eval_on_t+vb1*sin(q(2))];
%     figure
%     plot(x,y)
%     hold on
%     plot(node(:,1), node(:,2), 'b')
%     %     axis([0 30 0 30])
%     grid on
%     hold on
end

