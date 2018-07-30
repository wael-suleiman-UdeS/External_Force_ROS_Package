/******************************************************************************
 * @file    KalmanForceObserver.h
 * @authors  LH
 * @date    2017/12/20
 * @brief   An external force observer for humanoid robots. See our article :
 * Kalman Filter Based Observer for an External Force Applied to Medium-sized
 * Humanoid Robots
 ******************************************************************************/

#include <KalmanForceObserver.h>

KalmanForceObserver::KalmanForceObserver() :
    _reset(false), _forceOffset(1), _robot_mass(5.19)
{
  _TotalWeight_mean_filter = boost::shared_ptr<mean_filter<double> >(new mean_filter<double>);
  _LFCOP_mean_filter = boost::shared_ptr<mean_filter<double> >(new mean_filter<double>);
  _RFCOP_mean_filter = boost::shared_ptr<mean_filter<double> >(new mean_filter<double>);
  _schmitt_trig = boost::shared_ptr<schmitt_trigger<double> >(new schmitt_trigger<double>);
  _Filt_x = boost::shared_ptr<KalmanObserver>(new KalmanObserver);
  _Filt_y = boost::shared_ptr<KalmanObserver>(new KalmanObserver);
  _Filt_z = boost::shared_ptr<KalmanObserver_Z>(new KalmanObserver_Z);
  _ZMP.resize(2);
  _ZMPoffset(0) = 0;
  _ZMPoffset(1) = 0;
  _Xkoffset(0) = 0;
  _Xkoffset(1) = 0;
  _ACCoffset(0) = 0;
  _ACCoffset(1) = 0;
  _ACCoffset(2) = 0;
  
  _offsetTemp.resize(8);
}

void KalmanForceObserver::Init(KalmanForceObserver_ini ini, KalmanForceObserver_par par)
{
  _TotalWeight_mean_filter->set_window(par.TotalWeight_mean_filter_window);
  _LFCOP_mean_filter->set_window(par.LFCOP_mean_filter_window);
  _RFCOP_mean_filter->set_window(par.RFCOP_mean_filter_window);
  _schmitt_trig->set_threshold(par.Schmitt_negative_threshold, par.Schmitt_positive_threshold);
  _initialResetCounter = par.Reset_mean_filter_window;
  _resetCounter = _initialResetCounter - 1;
  _robot_mass = ini.robot_mass;

  // Initialize our filters
  KalmanObserver_ini FilterX_ini, FilterY_ini;
  KalmanObserver_Z_ini Filterz_ini;
  Filterz_ini.F_variance = ini.F_variance[2];
  Filterz_ini.acc_variance = ini.acc_variance[2];
  Filterz_ini.Period = ini.period;
  Filterz_ini.mass = ini.robot_mass;

  FilterX_ini.F_variance = ini.F_variance[0];
  FilterX_ini.acc_variance = ini.acc_variance[0];
  FilterX_ini.Period = ini.period;
  FilterX_ini.mass = ini.robot_mass;

  FilterY_ini.F_variance = ini.F_variance[1];
  FilterY_ini.acc_variance = ini.acc_variance[1];
  FilterY_ini.Period = ini.period;
  FilterY_ini.mass = ini.robot_mass;
  
  _forceOffset = ini.robot_mass * GRAV;

  double *P0 = &ini.P0[0];


  // Initial state are set to 0
  double X[5] = {0, 0, 0, 0, 0};
  _Filt_x->Init(X, P0, FilterX_ini);
  _Filt_y->Init(X, P0, FilterY_ini);

  // For Z, the position state is initialized to nao height
  double Z[5] = {0.315, 0, 0.0293, 0, 0};
  _Filt_z->Init(Z, P0, Filterz_ini);
}

void KalmanForceObserver::update_CoP(const std::vector<double> LFCoP,
                                     const std::vector<double> RFCoP)
{
  _LFCOP_mean_filter->add_data(LFCoP);
  _LFCOP_mean_filter->run_filter(_LFCOP);
  _RFCOP_mean_filter->add_data(RFCoP);
  _RFCOP_mean_filter->run_filter(_RFCOP);
}

void KalmanForceObserver::update_TotalWeightonFeet(std::vector<double> Weight)
{
  if (Weight[0] + Weight[1] < 1)
    return;
  _TotalWeight_mean_filter->add_data(Weight);
  _TotalWeight_mean_filter->run_filter(_TotalWeight);
}

void KalmanForceObserver::update_IMU(double acc[3], double gyro[3], double orientation[3])
{
  std::vector<double> ori;
  ori.push_back(orientation[1]);
  /// Gravity compensation
  double ax = acc[0] - 9.81 * sin(orientation[1]);
  ax = -ax * cos(orientation[1]);
  double ay = acc[1] - 9.81 * sin(orientation[0]);
  ay = -ay * cos(orientation[0]);
  _ACC(0) = ax;
  _ACC(1) = ay;
  _ACC(2) = acc[2];
}

void KalmanForceObserver::compute_zmp()
{
  double alpha = _TotalWeight.at(LEFT) / (_TotalWeight.at(LEFT) + _TotalWeight.at(RIGHT));

  _ZMP[0] = (1 - alpha) * (_RFCOP.at(0) + _RLegPos.at(0)) + (alpha) * (_LFCOP.at(0) + _LLegPos.at(0));
  _ZMP[1] = (1 - alpha) * (_RFCOP.at(1) + _RLegPos.at(1)) + (alpha) * (_LFCOP.at(1) + _LLegPos.at(1));
}

void KalmanForceObserver::compute_pivot_position()
{
  double trigger_signal = _TotalWeight.at(0) - _TotalWeight.at(1);

  if (_schmitt_trig->run(trigger_signal))
  {
    _Xk(0) = _LLegPos.at(0);
    _Xk(1) = _LLegPos.at(1);
  }
  else
  {
    _Xk(0) = _RLegPos.at(0);
    _Xk(1) = _RLegPos.at(1);
  }
}

void KalmanForceObserver::compute_pivot_position2()
{
  if (_TotalWeight.at(LEFT) + _TotalWeight.at(RIGHT) < 0.01)
    _TotalWeight.at(LEFT) = 1;

  double alpha = _TotalWeight.at(LEFT) / (_TotalWeight.at(LEFT) + _TotalWeight.at(RIGHT));
  _Xk(0) = alpha * (_LLegPos.at(0)) + (1 - alpha) * _RLegPos.at(0);
  _Xk(1) = alpha * (_LLegPos.at(1)) + (1 - alpha) * _RLegPos.at(1);
  _Xk(2) = alpha * (_LLegPos.at(2)) + (1 - alpha) * _RLegPos.at(2);
}

bool KalmanForceObserver::Run()
{
  if (is_Init())
  {
    if (_reset)
      reset_observer();

    compute_zmp();
    compute_pivot_position2();
    Step();
    return false;
  }

  return true;
}

void KalmanForceObserver::get_estimation(Eigen::Vector3d &F)
{
  F = _ForceEstimation;
}

void KalmanForceObserver::reset()
{
  _reset = true;
}

void KalmanForceObserver::update_COMPos(double com[3])
{
  _CoM_POS[0] = com[0];
  _CoM_POS[1] = com[1];
  _CoM_POS[2] = com[2];
}

void KalmanForceObserver::update_TorsoState(const std::vector<double> LLeg_state,
                                            const std::vector<double> RLeg_state)
{

  _RLegPos.resize(RLeg_state.size(), 0);
  _RLegPos = RLeg_state;
  _LLegPos.resize(LLeg_state.size(), 0);
  _LLegPos = LLeg_state;
}

void KalmanForceObserver::reset_observer()
{
  if(_resetCounter == _initialResetCounter - 1)
  {
      _resetCounter--;
      _offsetTemp.at(0) = _TotalWeight[0] + _TotalWeight[1];
      _offsetTemp.at(1) = _ZMP(0);
      _offsetTemp.at(2) = _ZMP(1);
      _offsetTemp.at(3) = _Xk(0);
      _offsetTemp.at(4) = _Xk(1);
      _offsetTemp.at(5) = _ACC(0);
      _offsetTemp.at(6) = _ACC(1);
      _offsetTemp.at(7) = _ACC(2);
  }
  else if(_resetCounter>0)
  {
      _resetCounter--;
      _offsetTemp.at(0) += _TotalWeight[0] + _TotalWeight[1];
      _offsetTemp.at(1) += _ZMP(0);
      _offsetTemp.at(2) += _ZMP(1);
      _offsetTemp.at(3) += _Xk(0);
      _offsetTemp.at(4) += _Xk(1);
      _offsetTemp.at(5) += _ACC(0);
      _offsetTemp.at(6) += _ACC(1);
      _offsetTemp.at(7) += _ACC(2);
      /*
      _forceOffset += _TotalWeight[0] + _TotalWeight[1];
      _ZMPoffset(0) += _ZMP(0);
      _ZMPoffset(1) += _ZMP(1);
      _Xkoffset(0) += _Xk(0);
      _Xkoffset(1) += _Xk(1);
      _ACCoffset(0) += _ACC(0);
      _ACCoffset(1) += _ACC(1);
      _ACCoffset(2) += _ACC(2);
      */
  }
  else
  {
    _resetCounter = _initialResetCounter - 1;
    _reset = false;
    _forceOffset  = (_offsetTemp.at(0) + _TotalWeight[0] + _TotalWeight[1]) / _initialResetCounter;
    _ZMPoffset(0) = (_offsetTemp.at(1) + _ZMP(0)) / _initialResetCounter;
    _ZMPoffset(1) = (_offsetTemp.at(2) + _ZMP(1)) / _initialResetCounter;
    _Xkoffset(0)  = (_offsetTemp.at(3) + _Xk(0)) / _initialResetCounter;
    _Xkoffset(1)  = (_offsetTemp.at(4) + _Xk(1)) / _initialResetCounter;
    _ACCoffset(0) = (_offsetTemp.at(5) + _ACC(0)) / _initialResetCounter;
    _ACCoffset(1) = (_offsetTemp.at(6) + _ACC(1)) / _initialResetCounter;
    _ACCoffset(2) = (_offsetTemp.at(7) + _ACC(2)) / _initialResetCounter;
  }
}

inline bool KalmanForceObserver::is_Init()
{

  if (_RLegPos.empty() || _LLegPos.empty() || _TotalWeight.empty())
    return false;

  return true;
}

void KalmanForceObserver::Step()
{
  RunfilterZ(); // Must run the Z filter first!
  RunfilterX();
  RunfilterY();
}

void KalmanForceObserver::RunfilterZ()
{

  std::vector<double> M, input;
  // Filter Z first
  M.push_back(double(_Xk(2)));
  M.push_back(double(_ACC(2) - _ACCoffset(2)));
  double f_o;
  if(_forceOffset > 0.0)
  {
      f_o = (-(_TotalWeight[0] + _TotalWeight[1]) * _robot_mass * GRAV) / _forceOffset; //-(_forceOffset + (_TotalWeight[0] + _TotalWeight[1])) * GRAV + _robot_mass * GRAV;
      M.push_back(f_o + _robot_mass * GRAV);
  }
  else
  {
      M.push_back(0.0);
  }
  _Filt_z->Run(input, M);
  _ForceEstimation(2) = _Filt_z->getForceEstimate();

  /// Update the parameters of the X and Y filters with the output
  std::vector<double> X = _Filt_z->getOutput();
  KalmanObserver_par par;

  par.FZ = _ForceEstimation(2);
  par.Z[0] = X[0];
  par.Z[1] = X[1];
  par.Z[2] = X[2];
  _Filt_x->updateParameters(par);
  _Filt_y->updateParameters(par);
}

void KalmanForceObserver::RunfilterY()
{
  std::vector<double> X, input;
  // Filter Y first
  X.push_back(double((_Xk(1) - _Xkoffset(1))));
  X.push_back(double(_ACC(1) - _ACCoffset(1)));
  X.push_back(double(_ZMP(1) - _ZMPoffset(1)));
  _Filt_y->Run(input, X);
  _ForceEstimation(1) = _Filt_y->getForceEstimate();
}

void KalmanForceObserver::RunfilterX()
{
  std::vector<double> X, input;
  // Filter X first
  X.push_back(double((_Xk(0) - _Xkoffset(0))));
  X.push_back(double(_ACC(0) - _ACCoffset(0)));
  X.push_back(double(_ZMP(0) - _ZMPoffset(0)));

  _Filt_x->Run(input, X);
  _ForceEstimation(0) = _Filt_x->getForceEstimate();
}

Eigen::Matrix3d KalmanForceObserver::euler2rotmat(double roll, double pitch, double yaw)
{
  Eigen::Matrix3d ROT_MAT;
  double cx = cos(roll);
  double sx = sin(roll);
  double cy = cos(pitch);
  double sy = sin(pitch);
  double cz = cos(yaw);
  double sz = sin(yaw);

  ROT_MAT << cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx, sz * cy, sz * sy * sx
      + cz * cz, sz * sy * cx - cz * sx, -sy, cy * sx, cy * cx;

  return ROT_MAT;
}
