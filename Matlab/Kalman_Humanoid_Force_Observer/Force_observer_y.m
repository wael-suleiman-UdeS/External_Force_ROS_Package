%Copyright 2017, Louis Hawley and Wael Suleiman

function sys_y = Force_observer_y( sys_y, Xc_Yc_Zc, acc, ZMP, sys_z, sigma_y, sigma_F, R_y, iter )

global dt g Mc

if iter==1,
sys_y.A= [1 dt dt^2/2 0 0; 0 1 dt 0 0; 0 0 1 0 0; 0 0 0 1 dt; 0 0 0 0 1];

B=[dt^3/6 0; dt^2/2 0; dt 0;0 dt^2/2; 0 dt];
sigma=[sigma_y 0; 0 sigma_F];
sys_y.Q= B*sigma*B';
    
sys_y.R= R_y;

sys_y.x=[Xc_Yc_Zc(1,2); 0; acc(1,2);0;0];
sys_y.P=1e2*eye(5);

sys_y.u=0;
sys_y.B=zeros(5,1);

end

Z_hat=sys_z(end).x;
z=Z_hat(1);
ddz=Z_hat(3);
Fz=Z_hat(4);

sys_y(end).H=[1 0 0 0 0; 0 0 1 0 0; 1 0 Mc*z/(-Mc*g-Mc*ddz+Fz) -z/(-Mc*g-Mc*ddz+Fz) 0];
sys_y(end).z = [Xc_Yc_Zc(2); acc(2); ZMP(2)]; 
sys_y(end+1)=kalman_filter(sys_y(end));


end

