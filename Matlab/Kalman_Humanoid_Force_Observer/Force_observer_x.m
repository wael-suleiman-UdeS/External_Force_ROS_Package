%Copyright 2017, Louis Hawley and Wael Suleiman
function sys_x = Force_observer_x( sys_x, Xc_Yc_Zc, acc, ZMP, sys_z, sigma_x, sigma_F, R_x, iter)

global dt g Mc

if iter==1,
sys_x.A= [1 dt dt^2/2 0 0; 0 1 dt 0 0; 0 0 1 0 0; 0 0 0 1 dt; 0 0 0 0 1];

B=[dt^3/6 0; dt^2/2 0; dt 0;0 dt^2/2; 0 dt];
sigma=[sigma_x 0; 0 sigma_F];
sys_x.Q= B*sigma*B';
    
sys_x.R= R_x;

sys_x.x=[Xc_Yc_Zc(1,1); 0; acc(1,1);0;0];
sys_x.P=1e2*eye(5);

sys_x.u=0;
sys_x.B=zeros(5,1);
end

Z_hat=sys_z(end).x;
z=  Z_hat(1);
ddz=Z_hat(3);
Fz= Z_hat(4);

sys_x(end).H=[1 0 0 0 0; 0 0 1 0 0; 1 0 Mc*z/(-Mc*g-Mc*ddz+Fz) -z/(-Mc*g-Mc*ddz+Fz) 0];
sys_x(end).z = [Xc_Yc_Zc(1); acc(1); ZMP(1)]; 
sys_x(end+1)=kalman_filter(sys_x(end));


end

