%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright 2018, Louis Hawley and Wael Suleiman
%
%  Kalman Filter Based Observer for an External Force Applied to 
%  Medium-sized Humanoid Robots
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global dt Zc g Mc
% load Acc.mat 
% load CoM_Pos.mat 
% load ZMP.mat
% load total_force.mat 
if ~exist('test_id','var')
    test_id = '';
end

dt=0.0167;
g=9.81;
Mc=5.19;

f_o=-(total_force (:,1)+total_force (:,2));

f_o= f_o*(Mc*g/abs(mean(f_o(1))));%normalizing the total force with the robot weight

figure,
plot(f_o);
title([test_id 'Total forces']);
grid

sigma_z = 1e3; sigma_x = sigma_z;sigma_y = sigma_z;

sigma_F_x=1e3;sigma_F_z=sigma_F_x; sigma_F_y=sigma_F_z; 

R_z=[0.01 0 0; 0 1 0; 0 0 1]; 
R_x=[0.01 0 0; 0 1 0; 0 0 0.01]; R_y=R_x;

sys_z=[]; sys_x=[]; sys_y=[];

for i=1:length(Xc_Yc_Zc),
    sys_z=Force_observer_z(sys_z, Xc_Yc_Zc(i,:), acc(i,:), f_o(i), sigma_z, sigma_F_z, R_z, i);
    sys_x=Force_observer_x(sys_x, Xc_Yc_Zc(i,:), acc(i,:), ZMP(i,:), sys_z, sigma_x, sigma_F_x, R_x, i);
    sys_y = Force_observer_y(sys_y, Xc_Yc_Zc(i,:), acc(i,:), ZMP(i,:), sys_z, sigma_y, sigma_F_y, R_y, i);
end

X_hat=[sys_x(1:end).x];
Y_hat=[sys_y(1:end).x];
Z_hat=[sys_z(1:end).x];

Fx=X_hat(4,:);
Fy=Y_hat(4,:);
Fz=Z_hat(4,:);
time=dt:dt:length(Fx)*dt;

SeaGreen=[46 139 87]/255;
Width2=1.5;
Width=0.5;
figure
plot(time,Fx,'b','LineWidth',Width)
grid
hold
plot(time,Fy,'k--','LineWidth',Width)
plot(time,Fz,'r-.','LineWidth',Width);
title([test_id 'Estimated Forces']);
plot(time,sqrt(Fx.^2+Fz.^2+Fy.^2),'Color',SeaGreen,'LineWidth',Width);
axis tight 
legend('F_x', 'F_y','F_z','||F_{ex}|| ', 'Location','northoutside','Orientation','horizontal');

xlabel('Time (s)');
ylabel('Force (N)');
