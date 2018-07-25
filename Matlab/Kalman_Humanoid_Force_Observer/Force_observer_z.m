%Copyright 2017, Louis Hawley and Wael Suleiman

function sys_z = Force_observer_z( sys_z, Xc_Yc_Zc, acc, f_o, sigma_z, sigma_F, Rz, iter )


global dt g Mc

if iter==1, %initialization 
sys_z.A= [1 dt dt^2/2 0 0; 0 1 dt 0 0; 0 0 1 0 0; 0 0 0 1 dt; 0 0 0 0 1];

sys_z.H=[1 0 0 0 0; 0 0 1 0 0; 0 0 -Mc 1 0];

B=[dt^3/6 0; dt^2/2 0; dt 0;0 dt^2/2; 0 dt];
sigma=[sigma_z 0; 0 sigma_F];
sys_z.Q= B*sigma*B';
    
sys_z.R= Rz;

sys_z.x=[Xc_Yc_Zc(1,3); 0; acc(1,3);0;0];
sys_z.P=1e2*eye(5);

sys_z.u=0;
sys_z.B=zeros(5,1);

end
    
    sys_z(end).z = [Xc_Yc_Zc(3); acc(3); (f_o+Mc*g)]; 
    sys_z(end+1)=kalman_filter(sys_z(end));


end

