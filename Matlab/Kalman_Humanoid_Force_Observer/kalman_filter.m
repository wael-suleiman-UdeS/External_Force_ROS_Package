%Copyright 2017, Louis Hawley and Wael Suleiman

function sys = kalman_filter(sys)
   
   % Prediction step:
   sys.x = sys.A*sys.x + sys.B*sys.u;
   sys.P = sys.A * sys.P * sys.A' + sys.Q;

   % Kalman matrix gain:
   K = sys.P*sys.H'*inv(sys.H*sys.P*sys.H'+sys.R);

   % Correction step:
   sys.x = sys.x + K*(sys.z-sys.H*sys.x);
   sys.P = sys.P - K*sys.H*sys.P;

return