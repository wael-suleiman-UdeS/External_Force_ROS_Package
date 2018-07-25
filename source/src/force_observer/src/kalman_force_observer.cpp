/******************************************************************************
 * @file    kalman_force_observer.cpp
 * @authors  LH
 * @date    2017/12/20
 * @brief   Kalman force observer node
 ******************************************************************************/
#include <KalmanForceObserverNode.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Kalman_Force_observer_node");


  KalmanForceObserverNode node;

  node.run();

  return 0;
}



