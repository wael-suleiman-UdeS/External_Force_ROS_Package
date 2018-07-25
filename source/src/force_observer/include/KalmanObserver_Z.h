/******************************************************************************
 * @file    ForceObserverKalman_X.h
 * @authors  LH
 * @date    2017/12/18
 * @brief   A particular Implementation of a kalman-filter based force observer
 ******************************************************************************/

#ifndef HUMANOID_CONTROL_TRO_FORCE_OBSERVER_INCLUDE_KALMANOBSERVER_Z_H_
#define HUMANOID_CONTROL_TRO_FORCE_OBSERVER_INCLUDE_KALMANOBSERVER_Z_H_

#include "ekalman_filter.h"

struct KalmanObserver_Z_ini
{
  double F_variance;

  double acc_variance;

  double Period;

  double mass;
};

class KalmanObserver_Z : public EKalmanFilter
{

public:
  KalmanObserver_Z();
  ~KalmanObserver_Z();

  /*********************************************************
   * @brief Get the force estimation
   *
   * @return The force estimation (N)
   *********************************************************/
  double getForceEstimate();

  void Init(const double initial_state[], const double P0_ini[], KalmanObserver_Z_ini ini);

protected:
  void makeBaseH();
  void makeBaseQ();
  void makeBaseR();

private:
  // Variables that are specific to a particular implementation
  double _F_variance, _acc_variance;
  double Mc;
};

#endif /* HUMANOID_CONTROL_TRO_FORCE_OBSERVER_INCLUDE_KALMANOBSERVER_Z_H_ */
