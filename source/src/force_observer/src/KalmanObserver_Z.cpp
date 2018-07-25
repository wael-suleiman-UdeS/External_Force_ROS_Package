/******************************************************************************
 * @file    KalmanObserver_Z.cpp
 * @authors  LH
 * @date    2017/12/19
 * @brief   Implementation of kalman-filter based force observer
 ******************************************************************************/

#include <KalmanObserver_Z.h>
#include <cmath>
#include <iostream>

using namespace std;
using namespace Kalman;

KalmanObserver_Z::KalmanObserver_Z() :
    Mc(5.19), EKalmanFilter(5, 0, 5, 3, 3),_acc_variance(1000), _F_variance(1000)
{
}

KalmanObserver_Z::~KalmanObserver_Z()
{
}

double KalmanObserver_Z::getForceEstimate()
{
  return _output(4);
}

void KalmanObserver_Z::Init(const double initial_state[], const double P0_ini[], KalmanObserver_Z_ini ini)
{

  Mc = ini.mass;
  _acc_variance = ini.acc_variance;
  _F_variance = ini.F_variance;
  setPeriod(ini.Period);
  Initialize(initial_state, P0_ini);
}

void KalmanObserver_Z::makeBaseH()
{

  H(1, 1) = 1.0;
  H(1, 2) = 0.0;
  H(1, 3) = 0.0;
  H(1, 4) = 0.0;
  H(1, 5) = 0.0;

  H(2, 1) = 0.0;
  H(2, 2) = 0.0;
  H(2, 3) = 1.0;
  H(2, 4) = 0.0;
  H(2, 5) = 0.0;

  H(3, 1) = 0.0;
  H(3, 2) = 0.0;
  H(3, 3) = -Mc;
  H(3, 4) = 1.0;
  H(3, 5) = 0.0;
}

void KalmanObserver_Z::makeBaseQ()
{
  Eigen::Matrix<double, 5, 2> BB;
  BB(0, 0) = period3 / 6;
  BB(0, 1) = 0;
  BB(1, 0) = period2 / 2;
  BB(1, 1) = 0;
  BB(2, 0) = Period;
  BB(2, 1) = 0;
  BB(3, 0) = 0;
  BB(3, 1) = period2 / 2;
  BB(4, 0) = 0;
  BB(4, 1) = Period;

  Eigen::Matrix<double, 2, 2> temp;
  temp(0, 0) = _acc_variance;
  temp(0, 1) = 0;
  temp(1, 0) = 0;
  temp(1, 1) = _F_variance;

  Eigen::Matrix<double, 5, 5> QQ = BB * temp * BB.transpose();
  Q(1, 1) = QQ(0, 0);
  Q(1, 2) = QQ(0, 1);
  Q(1, 3) = QQ(0, 2);
  Q(1, 4) = QQ(0, 3);
  Q(1, 5) = QQ(0, 4);

  Q(2, 1) = QQ(1, 0);
  Q(2, 2) = QQ(1, 1);
  Q(2, 3) = QQ(1, 2);
  Q(2, 4) = QQ(1, 3);
  Q(2, 5) = QQ(1, 4);

  Q(3, 1) = QQ(2, 0);
  Q(3, 2) = QQ(2, 1);
  Q(3, 3) = QQ(2, 2);
  Q(3, 4) = QQ(2, 3);
  Q(3, 5) = QQ(2, 4);

  Q(4, 1) = QQ(3, 0);
  Q(4, 2) = QQ(3, 1);
  Q(4, 3) = QQ(3, 2);
  Q(4, 4) = QQ(3, 3);
  Q(4, 5) = QQ(3, 4);

  Q(5, 1) = QQ(4, 0);
  Q(5, 2) = QQ(4, 1);
  Q(5, 3) = QQ(4, 2);
  Q(5, 4) = QQ(4, 3);
  Q(5, 5) = QQ(4, 4);

}

void KalmanObserver_Z::makeBaseR()
{
  R(1, 1) = 0.01;
  R(1, 2) = 0.0;
  R(1, 3) = 0.0;

  R(2, 1) = 0.0;
  R(2, 2) = 1.00;
  R(2, 3) = 0.0;

  R(3, 1) = 0.0;
  R(3, 2) = 0.0;
  R(3, 3) = 0.01;
}
