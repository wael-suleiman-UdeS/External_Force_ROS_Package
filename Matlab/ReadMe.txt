#Experimental data accompanying the paper: "Kalman Filter Based Observer for an External Force Applied to Medium-sized Humanoid Robots", submitted to IROS 2018

To run the force observer under MATLAB:

1) Run the script RunAll.m 

or

1)- Add the folder "Kalman_Humanoid_Force_Observer" to MATLAB path

2)- Each test folder, for instance "Force_Z_data", contains the measured data of:
    i)- CoM position [Xc, Yc, Zc] in "CoM_Pos.mat"
    ii)- The force applied on both feet in "total_force.mat"
    iii)- The ZMP=[ZMPx, ZMPy] in "ZMP.mat"
    iv)- The acceleration [ddx,ddy,ddz] in "Acc.mat"

3)- In each folder, load all the datas (i.e. load('Acc.mat')) then run the MATLAB command:
Kalman_filter_force_observer
