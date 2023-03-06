function [Pul_pri_mot_r] = Separador_Comas_Arduino(vect_dir_pri_mot,Pul_pri_mot_r)
    %Paso quien sabe: Separar el vector por comas para meterlo en el arduino
    %Para un solo motor
    vect_dir_pri_mot = sprintf('%.0f,' , vect_dir_pri_mot);
    vect_dir_pri_mot = vect_dir_pri_mot(1:end-1);% strip final comma
    Pul_pri_mot_r = abs(Pul_pri_mot_r);
    Pul_pri_mot_r = sprintf('%.0f,' , Pul_pri_mot_r);
    Pul_pri_mot_r = Pul_pri_mot_r(1:end-1);% strip final comma
    
end

