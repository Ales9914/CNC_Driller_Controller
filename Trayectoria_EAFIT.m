%Aproximación por deltas
clc
clear
L = 0.2476;
[x,y] = Nube_EAFIT(14*L/10,L/2,100,0.075);
[~,size_x] = size(x);
L = 0.2476;
vb1 = 0.255;
vb2 = 0.255;
A = (vb1^2-vb2^2+L^2)/2;
Speed_factor = 50;
step = 1;
step = step/Speed_factor;
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

d_h = y-vp1 - sqrt(-2.*A + (y-vp1).^2 + 2.*L.*sqrt(vb1.^2 - (y-vp1).^2));
vp2 = vp1 + d_h;
[size_pr_vp2] = size(find(vp2<0));
if size_pr_vp2 ~= 0
    disp("Second scroll lower than 0")
end

%Aproximación
vel_arr = []; %T1 T2
ace_arr = [];
for i = 1:size_x-1
    vel_arr(1,i) = (vp1(i+1)-vp1(i))/step;
    vel_arr(2,i) = (vp2(i+1)-vp2(i))/step;
end
vel_arr(1,size_x) = vel_arr(1,size_x-1);
vel_arr(2,size_x) = vel_arr(2,size_x-1);
for i = 1:size_x-1
    ace_arr(1,i) = (vel_arr(1,i+1)-vel_arr(1,i))/step;
    ace_arr(2,i) = (vel_arr(2,i+1)-vel_arr(2,i))/step;
end
ace_arr(1,size_x) = ace_arr(1,size_x-1);
ace_arr(2,size_x) = ace_arr(2,size_x-1);


step = 1:1:size_x;
step = step/Speed_factor;
figure
plot(step',vp1')
grid on
hold on
plot(step',vel_arr(1,:)')
plot(step',ace_arr(1,:)')

figure
plot(step',vp2')
grid on
hold on
plot(step',vel_arr(2,:)')
plot(step,ace_arr(2,:))

Ts_arr = [vp1;
          vel_arr(1,:);
          ace_arr(1,:);
          vp2;
          vel_arr(2,:);
          ace_arr(2,:)];
    
    
    