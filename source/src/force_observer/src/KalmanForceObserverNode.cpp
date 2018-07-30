/******************************************************************************
 * @file    KalmanForceObserverNode.cpp
 * @authors  LH
 * @date    2016/09/21
 * @brief   ROS interface to Kalman-based force observer
 ******************************************************************************/
#include <KalmanForceObserverNode.h>

KalmanForceObserverNode::KalmanForceObserverNode():_init(false)
{

  ros::NodeHandle nh_private("~");
  ros::NodeHandle nh;

  KalmanForceObserver_ini ini;
  KalmanForceObserver_par par;
  nh_private.getParam("Mean_filter_window/TotalWeight", par.TotalWeight_mean_filter_window);
  nh_private.getParam("Mean_filter_window/LFCOP", par.LFCOP_mean_filter_window);
  nh_private.getParam("Mean_filter_window/RFCOP", par.RFCOP_mean_filter_window);
  nh_private.getParam("Mean_filter_window/Reset", par.Reset_mean_filter_window);
  nh_private.getParam("Force_Variance", ini.F_variance);
  nh_private.getParam("Acceleration_Variance", ini.acc_variance);
  nh_private.getParam("Covariance_Matrix", ini.P0);

  double Rate;
  nh_private.param<double>("Kalman_Rate", _rate, 60);
  ini.period = 1 / _rate;
  nh_private.param<double>("RobotMass", ini.robot_mass, 5.19);
  par.Schmitt_negative_threshold = -2;
  par.Schmitt_positive_threshold = 2;

  _imu_sub = nh.subscribe<sensor_msgs::Imu>("imu", 1, boost::bind(&KalmanForceObserverNode::imu_callback, this, _1));
  _fsr_sub = nh.subscribe<force_observer_msgs::FSR>("fsr", 1, boost::bind(&KalmanForceObserverNode::fsr_callback, this, _1));
  _PelvisPos_state_sub = nh.subscribe<force_observer_msgs::PelvisPosition>(
      "pelvis", 1, boost::bind(&KalmanForceObserverNode::PelvisPos_callback, this, _1));
  _CoM_state_sub = nh.subscribe<force_observer_msgs::COM>(
      "CoM_pos", 1, boost::bind(&KalmanForceObserverNode::CoM_callback, this, _1));
  _resetFilterService = nh.advertiseService("Reset_observer",
                                            &KalmanForceObserverNode::reset_observer, this);

  _ForceEstimationpub = nh.advertise<geometry_msgs::Wrench>("ext_force_kalman", 1);


  _observer = boost::shared_ptr<KalmanForceObserver>(new KalmanForceObserver);
  _observer->Init(ini, par);
  ros::Duration sleep_time(5);
  sleep_time.sleep();
  _init = true;

}

void KalmanForceObserverNode::run()
{

  ros::Rate loop_rate(_rate);

  geometry_msgs::Wrench msg;
  geometry_msgs::Point msg_zmp;
  
  Eigen::Vector3d datas;
  Eigen::Vector2d zmp_est;
  while (ros::ok())
  {
    ros::spinOnce();
    {
      if (_observer->Run())
        continue;
      _observer->get_estimation(datas);
      msg.force.x = datas(0);
      msg.force.y = datas(1);
      msg.force.z = datas(2);
      _ForceEstimationpub.publish(msg);
    }

    loop_rate.sleep();
    if (loop_rate.cycleTime() > ros::Duration(0.013))
      ROS_WARN(
          "Kalman force observer loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds",
          60.0, loop_rate.cycleTime().toSec());
  }

}

void KalmanForceObserverNode::imu_callback(const sensor_msgs::ImuConstPtr msg)
{

  if(!_init)
    return;
  /// Check the values are real numbers
  double check_value = msg->linear_acceleration.x * msg->linear_acceleration.y
      * msg->linear_acceleration.z * msg->angular_velocity.x * msg->angular_velocity.y
      * msg->angular_velocity.z * msg->orientation.x * msg->orientation.y * msg->orientation.z
      * msg->orientation.w;

  if (std::isnan(check_value))
    return;

  double acc[3];
  double gyro[3];
  double orientation[3];
  acc[0] = msg->linear_acceleration.x;
  acc[1] = msg->linear_acceleration.y;
  acc[2] = msg->linear_acceleration.z;
  gyro[0] = msg->angular_velocity.x;
  gyro[1] = msg->angular_velocity.y;
  gyro[2] = msg->angular_velocity.z;
  // Convert orientation to RPY
  Eigen::Quaterniond quat(msg->orientation.w, msg->orientation.x, msg->orientation.y,
                          msg->orientation.z);
  Eigen::Vector3d rpy = quat.toRotationMatrix().eulerAngles(2, 1, 0);
  orientation[0] = rpy(0);
  orientation[1] = rpy(1);
  orientation[2] = rpy(2);

  _observer->update_IMU(acc, gyro, orientation);

}

void KalmanForceObserverNode::fsr_callback(const force_observer_msgs::FSRConstPtr msg)
{
  if(!_init)
    return;

  /// Check the values are real numbers
  double check_value = msg->LF_COP.elems[0] * msg->LF_COP.elems[1] * msg->RF_COP.elems[0]
      * msg->RF_COP.elems[1] * msg->left_foot_total * msg->right_foot_total;
  if (std::isnan(check_value))
    return;

  std::vector<double> LF_CoP, RF_CoP, Weight_on_feet;
  LF_CoP.push_back(double(msg->LF_COP.elems[0]));
  LF_CoP.push_back(double(msg->LF_COP.elems[1]));
  RF_CoP.push_back(double(msg->RF_COP.elems[0]));
  RF_CoP.push_back(double(msg->RF_COP.elems[1]));
  Weight_on_feet.push_back(msg->left_foot_total);
  Weight_on_feet.push_back(msg->right_foot_total);

  _observer->update_CoP(LF_CoP, RF_CoP);
  _observer->update_TotalWeightonFeet(Weight_on_feet);
}

void KalmanForceObserverNode::PelvisPos_callback(
    const force_observer_msgs::PelvisPositionConstPtr msg)
{

  if(!_init)
    return;

  double check_value = msg->leftfoot.position.x * msg->leftfoot.position.y
      * msg->leftfoot.position.z * msg->rightfoot.position.x * msg->rightfoot.position.y
      * msg->rightfoot.position.z * msg->leftfoot.orientation.x * msg->leftfoot.orientation.y
      * msg->leftfoot.orientation.z * msg->leftfoot.orientation.w * msg->rightfoot.orientation.x
      * msg->rightfoot.orientation.y * msg->rightfoot.orientation.z * msg->rightfoot.orientation.w;

  if (std::isnan(check_value))
    return;

  std::vector<double> LLeg_state, RLeg_state;
  LLeg_state.push_back(-msg->leftfoot.position.x);
  LLeg_state.push_back(-msg->leftfoot.position.y);
  LLeg_state.push_back(-msg->leftfoot.position.z);
  RLeg_state.push_back(-msg->rightfoot.position.x);
  RLeg_state.push_back(-msg->rightfoot.position.y);
  RLeg_state.push_back(-msg->rightfoot.position.z);
  LLeg_state.push_back(msg->leftfoot.orientation.x);
  LLeg_state.push_back(msg->leftfoot.orientation.y);
  LLeg_state.push_back(msg->leftfoot.orientation.z);
  LLeg_state.push_back(msg->leftfoot.orientation.w);
  RLeg_state.push_back(msg->rightfoot.orientation.x);
  RLeg_state.push_back(msg->rightfoot.orientation.y);
  RLeg_state.push_back(msg->rightfoot.orientation.z);
  RLeg_state.push_back(msg->rightfoot.orientation.w);

  _observer->update_TorsoState(LLeg_state, RLeg_state);
}

void KalmanForceObserverNode::CoM_callback(const force_observer_msgs::COMConstPtr msg)
{
  double COM[3] = {msg->CoM.position.x, msg->CoM.position.y, msg->CoM.position.z};
  _observer->update_COMPos(COM);
}

bool KalmanForceObserverNode::reset_observer(std_srvs::EmptyRequest &req,
                                             std_srvs::EmptyResponse &resp)
{
  _observer->reset();
  return true;
}


