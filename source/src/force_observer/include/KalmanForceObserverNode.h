/******************************************************************************
 * @file    KalmanForceObserverNode.h
 * @authors  LH
 * @date    2017/12/20
 * @brief   ROS Interface to Kalman Filter Based Observer
 ******************************************************************************/

#ifndef HUMANOID_CONTROL_TRO_FORCE_OBSERVER_INCLUDE_KALMANFORCEOBSERVERNODE_H_
#define HUMANOID_CONTROL_TRO_FORCE_OBSERVER_INCLUDE_KALMANFORCEOBSERVERNODE_H_

/// Project Include
#include <KalmanForceObserver.h>

/// ROS Include
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Point.h>
#include "force_observer_msgs/FSR.h"
#include "force_observer_msgs/COM.h"
#include "force_observer_msgs/PelvisPosition.h"
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

#include <std_msgs/Float64MultiArray.h>

/// C++ Include
#include <eigen3/Eigen/Core>
#include <math.h>

class KalmanForceObserverNode
{
public:

  KalmanForceObserverNode();

  /*******************************************************************
   * @brief Starts the Force observer node
   *
   * This is a blocking function. It starts a loop that updates the
   * force estimation at a constant rate.
   *******************************************************************/
  void run();

private:

  void imu_callback(const sensor_msgs::ImuConstPtr msg);

  void fsr_callback(const force_observer_msgs::FSRConstPtr msg);

  void PelvisPos_callback(const force_observer_msgs::PelvisPositionConstPtr msg);

  void CoM_callback(const force_observer_msgs::COMConstPtr msg);

  bool reset_observer(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &resp);


  /// ROS Subscriber and ServiceServer
  ros::Subscriber _imu_sub;
  ros::Subscriber _fsr_sub;
  ros::Subscriber _CoM_state_sub;
  ros::Subscriber _PelvisPos_state_sub;
  ros::ServiceServer _resetFilterService;

  /// Force Estimation Publisher
  ros::Publisher _ForceEstimationpub;

  boost::shared_ptr<KalmanForceObserver> _observer;

  double _rate;
  bool _init;
};

#endif /* HUMANOID_CONTROL_TRO_FORCE_OBSERVER_INCLUDE_KALMANFORCEOBSERVERNODE_H_ */
