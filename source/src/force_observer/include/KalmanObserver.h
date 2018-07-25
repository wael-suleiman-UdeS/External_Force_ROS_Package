/******************************************************************************
 * @file    ForceObserverKalman_X.h
 * @authors  LH
 * @date    2017/12/18
 * @brief   Implementation of kalman-filter based force observer
 ******************************************************************************/

#ifndef HUMANOID_CONTROL_TRO_FORCE_OBSERVER_INCLUDE_KALMANOBSERVER_H_
#define HUMANOID_CONTROL_TRO_FORCE_OBSERVER_INCLUDE_KALMANOBSERVER_H_

#include "ekalman_filter.h"

/**************************************************************
 * @brief Dynamic parameters
 *  These parameters are used in the modelisation of the system
 *   dynamic. They can be modified during run-time by calling
 *   the updateParameters() function.
 **************************************************************/
struct KalmanObserver_par
{
  double FZ;
  double Z[3];
};

/**************************************************************
 *  @brief Initialisation parameters
 *  These parameters are used once to initialize the filters.
 *  They cannot be modified later
 **************************************************************/
struct KalmanObserver_ini
{
  /// Parameters used to set the process variance
  double F_variance; /// The variance of the force
  double acc_variance; /// The variance of the acceleration

  double Period; /// Filter update period

  double mass; /// Mass of the robot. Used in the dynamic modelisation
};

class KalmanObserver : public EKalmanFilter
{

public:
  KalmanObserver();
  ~KalmanObserver();

  /**************************************************************
   * @brief Updates the state  of the torso on the z axis
   *        This information is needed by the x-axis dynamic model
   *
   * @param Z [in] Array of 3 element (z , z_dot, and z_ddot)
   **************************************************************/
  void updateZ(double Z[3]);

  /*********************************************************
   * @brief Updates the variable parameters of the dynamic model
   *        This function must be called before running the
   *        filters everytime the parameters changes.
   * @param par [in] The variable parameters
   *********************************************************/
  void updateParameters(KalmanObserver_par par);

  /*********************************************************
   * @brief Get the force estimation
   *
   * @return The force estimation (N)
   *********************************************************/
  double getForceEstimate();

  void Init(const double initial_state[], const double P0_ini[], KalmanObserver_ini ini);

  void setInitialState(double initial_state[], double P0_ini[]);

protected:

  void makeH();

  void makeBaseH();

  void makeBaseQ();

  void makeBaseR();
private:

  // Variables that are specific to a particular implementation
  double _F_variance, _acc_variance;
  double Mc, FZ;
  double Zc[3];
};

#endif /* HUMANOID_CONTROL_TRO_FORCE_OBSERVER_INCLUDE_KALMANOBSERVER_H_ */
