/******************************************************************************
 * @file    KalmanForceObserver.h
 * @authors  LH
 * @date    2017/12/20
 * @brief   An external force observer for humanoid robots. See our article :
 * Kalman Filter Based Observer for an External Force Applied to Medium-sized
 * Humanoid Robots
 ******************************************************************************/

#ifndef HUMANOID_CONTROL_TRO_FORCE_OBSERVER_INCLUDE_KALMANFORCEOBSERVER_H_
#define HUMANOID_CONTROL_TRO_FORCE_OBSERVER_INCLUDE_KALMANFORCEOBSERVER_H_

/// Project Include
#include <KalmanObserver.h>
#include <KalmanObserver_Z.h>
#include <Mean_filter.h>
#include <Schmitt_trigger.h>
#include <General_Filter.h>

/// C++ Include
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <math.h>
#include <boost/shared_ptr.hpp>

#define GRAV 9.81

typedef double format;

enum Foot
{
  LEFT, RIGHT
};

struct KalmanForceObserver_ini
{
  double period;

  double robot_mass;

  std::vector<double> acc_variance;

  std::vector<double> F_variance;

  // Initial covariance matrixx
  std::vector<double> P0;
};

struct KalmanForceObserver_par
{
  int LFCOP_mean_filter_window;

  int RFCOP_mean_filter_window;

  int TotalWeight_mean_filter_window;
  
  int Reset_mean_filter_window;

  double Schmitt_positive_threshold;

  double Schmitt_negative_threshold;
};

class KalmanForceObserver
{

public:
  KalmanForceObserver();

  /*******************************************************************
   * @brief Initialisation function
   * @param[in] ini The initialisation parameters
   * @param[in] par The class parameters
   *******************************************************************/
  void Init(KalmanForceObserver_ini ini, KalmanForceObserver_par par);


  /*******************************************************************
   * @brief Class main function. Should be call at a regular interval
   *******************************************************************/
  bool Run();


  /*******************************************************************
   * @brief Returns the force estimation
   * @param[out] Vector containing the external force estimates (x,y,z)
   *******************************************************************/
  void get_estimation(Eigen::Vector3d &ForceEstimation);

  /*******************************************************************
   * @brief Resets the force observer. This function should always
   * be called at the beginning, when the robot is standing still and
   * no force is applied. This will create reference value so that
   * the system can detect when a force is exerted on the robot.
   *******************************************************************/
  void reset();


  /*******************************************************************
   * @brief Checks if the observer can be runned (i.e. If enough datas
   *        are available.
   * @return
   *******************************************************************/
  bool is_Init();


  /// Datas update functions
  /*******************************************************************
   * @brief Update the center of pressure under each foot
   * @param[in] LFCoP  Vector 2D containing the center of pressure of the
   *                                     left foot [x,y]
   * @param[in] RFCoP  Vector 2D containing the center of pressure of the
   *                                     right foot [x,y]
   *******************************************************************/
  void update_CoP(const std::vector<format> LFCoP, const std::vector<format> RFCoP);

  /*******************************************************************
   * @brief Update the force on each foot
   * @param[in] weight Vector 2D containing the weight detected on each
   *                             foot [Left foot, Right Foot];
   *******************************************************************/
  void update_TotalWeightonFeet(const std::vector<format> weight);

  /*******************************************************************
   * @brief Update the position of the torso with respect to each foot
   * @param[in] _LLegPos Vector 2D containing the position of the torso
   *                                     with respect to the left foot [x,y,z]
   * @param[in] _RLegPos Vector 2D containing the position of the torso
   *                                     with respect to the right foot [x,y,z]
   *******************************************************************/
  void update_TorsoState(std::vector<format> _LLegPos, std::vector<format> _RLegPos);

  /*******************************************************************
   * @brief Update the datas incoming from the IMU
   * @param[in] acc Acceleration(x,y,z) (m/s^2)
   * @param[in] gyro Angular speed (x,y) rad/s
   * @param[in] orientation The orientation of the torso (RPY).
   *******************************************************************/
  void update_IMU(double acc[3], double gyro[3], double orientation[3]);

  /*******************************************************************
   * @brief Update the position of the Center of Mass with respect to
   *              the robot torso.
   * @param[in] COM  CoM position relative to torso (x,y) (m)
   *******************************************************************/
  void update_COMPos(double COM[3]);


private:

  /*******************************************************************
   * @brief Executes a step of each kalman filter
   *
   *******************************************************************/
  void Step();

  void RunfilterX();
  void RunfilterY();
  void RunfilterZ();

  void compute_zmp();

  //------ Pivot position computation ------//
  // As explained in the article, there is 2 ways of computing the
  // position of the pivot of the inverted pendulum. compute_pivot_position() uses the
  //schmitt_trigger and the other ones uses a weighted-average of the position of the ankles
  // using the total weight on each foot
  void compute_pivot_position();
  void compute_pivot_position2();

  void median_filter();

  void low_pass_filter();

  void reset_observer();

  Eigen::Matrix3d euler2rotmat(double roll, double pitch, double yaw);

  /// Class Attributes
  bool _reset;

  /// Filters for each axis
  boost::shared_ptr<KalmanObserver> _Filt_x;
  boost::shared_ptr<KalmanObserver> _Filt_y;
  boost::shared_ptr<KalmanObserver_Z> _Filt_z;

  /// Mean filter object
  boost::shared_ptr<mean_filter<format> > _LFCOP_mean_filter;
  boost::shared_ptr<mean_filter<format> > _RFCOP_mean_filter;
  boost::shared_ptr<mean_filter<format> > _TotalWeight_mean_filter;

  std::vector<format> _LFCOP, _RFCOP;
  std::vector<format> _orientationX;
  std::vector<format> _TotalWeight;
  std::vector<format> _LLegPos, _RLegPos;

  Eigen::Vector2d _ZMP, _ZMPoffset;
  Eigen::Vector3d _Xk, _Xkoffset;
  Eigen::Vector3d _CoM_POS;
  Eigen::Vector3d _ACC, _ACCoffset;
  Eigen::Vector3d _ForceEstimation;
  
  std::vector<double> _offsetTemp;
  
  int _resetCounter;
  int _initialResetCounter;

  boost::shared_ptr<schmitt_trigger<format> > _schmitt_trig;

  /// @brief A constant : COM/g
  double _forceOffset;
  double _robot_mass;

  std::vector<format> _Result;

};

#endif /* HUMANOID_CONTROL_TRO_FORCE_OBSERVER_INCLUDE_KALMANFORCEOBSERVER_H_ */
