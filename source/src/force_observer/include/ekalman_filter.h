/******************************************************************************
 * @file    kalman_filter.h
 * @authors  LH
 * @date    2017/12/17
 * @brief   Common Kalman filter Interface.
 *          This class provides interfaces to an EKF. It also implements the common
 *          function for setting matrix that are not useful for a linear KF.
 *          Function makeBaseA() and makeBaseH() must be overloaded to specify
 *          the system dynamic.
 ******************************************************************************/
#ifndef HUMANOID_NAVIGATION_CONTROLLER_WALK_CONTROLLER_INCLUDE_EKALMAN_FILT_H_
#define HUMANOID_NAVIGATION_CONTROLLER_WALK_CONTROLLER_INCLUDE_EKALMAN_FILT_H_

#include "ekfilter.hpp"
#include <Eigen/Core>


class EKalmanFilter : public Kalman::EKFilter<double, 1, false, false, true>
{
public:

  /*********************************************************
   * @brief Filter creator
   * @param n [in] Size of the state vector
   * @param nu [in] Size of the input vector
   * @param nw [in] Size of the process noise vector
   * @param m [in] Size of the measurement vector
   * @param nv [in] Size of the measurement noise vector
   *********************************************************/
  EKalmanFilter(int n,int nu,int nw,int m,int nv);

  /*********************************************************
   * @brief Set the sampling period
   *
   * @param  period [in] Sampling period in seconds
   *********************************************************/
  void setPeriod(double sampling_period);

  /*********************************************************
   * @brief Execute a single filtering step
   *
   * @param input [in] Input to the system
   *
   * @param measure [in] Measured position, acceleration and zmp_x
   *********************************************************/
  void Run(std::vector<double> input, std::vector<double> measure);

  /*********************************************************
   * @brief Get the state estimation
   *
   * @return The output of the filter
   *********************************************************/
  std::vector<double> getOutput();


protected:
  /*********************************************************
   * @brief Initialize the Filter with the initial state
   *
   * @param initial_state [in] The initial state of the
   * system (x, x_dot, x_dot_dot, F_x, F_x_dot)(m,s)
   *
   * @param P0 [in] Initial error covariance matrix
   *********************************************************/
  void Initialize(const double initial_state[], const double P0_ini[]);
  virtual void makeBaseA();
  virtual void makeBaseH();
  void makeBaseV();
  virtual void makeBaseR();
  void makeBaseW();
  virtual void makeBaseQ();
  void makeProcess();
  void makeMeasure();

  double Period, Gravity;
  double period6, period5, period4, period3, period2;

  /// @brief FIlter object
  EKalmanFilter::Vector _input;
  EKalmanFilter::Vector _measure;
  EKalmanFilter::Vector _output;

};

typedef EKalmanFilter::Vector Vector;
typedef EKalmanFilter::Matrix Matrix;

#endif /* HUMANOID_NAVIGATION_CONTROLLER_WALK_CONTROLLER_INCLUDE_EKALMAN_FILT_H_ */
