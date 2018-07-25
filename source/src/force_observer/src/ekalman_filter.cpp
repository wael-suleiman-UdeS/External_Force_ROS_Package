// -------------- kalman_filt.cpp - Kalman filter ------------------------//
//
// This file is part of kfilter.
// kfilter is a C++ variable-dimension extended kalman filter library.
//
// Copyright (C) 2004        Vincent Zalzal, Sylvain Marleau
// Copyright (C) 2001, 2004  Richard Gourdeau
// Copyright (C) 2004        GRPR and DGE's Automation sector
//                           École Polytechnique de Montréal
//
// Code adapted from algorithms presented in :
//      Bierman, G. J. "Factorization Methods for Discrete Sequential
//      Estimation", Academic Press, 1977.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
// ---------------------------  Kalman filter ------------------------//
/*
 TODO Documented the modeled dynamics
 */

#include "ekalman_filter.h"
#include <cmath>
#include <iostream>

using namespace std;
EKalmanFilter::EKalmanFilter(int n, int nu, int nw, int m, int nv_)
{
  setDim(n, nu, nw, m, nv_);
  _input.resize(nu);
  _output.resize(n);
  _measure.resize(m);
  Period = 0.01667; /// Defaults to 60 hz
  Gravity = 9.81;
  period2 = Period * Period;
  period3 = period2 * Period;
  period4 = period3 * Period;
  period5 = period4 * Period;
  period6 = period5 * Period;
}

void EKalmanFilter::Run(std::vector<double> input, std::vector<double> measure)
{
  for (int I = 1; I <= _measure.size(); I++)
  {
    _measure(I) = measure.at(I - 1);
  }

  for (int I = 1; I <= _input.size(); I++)
  {
    _input(I) = input.at(I - 1);
  }

  step(_input, _measure);
  _output = getX();
}

std::vector<double> EKalmanFilter::getOutput()
{
  std::vector<double> output;
  for (int I = 1; I <= _output.size(); I++)
    output.push_back(_output(I));

  return output;
}

void EKalmanFilter::Initialize(const double initial_state[],const double P0_ini[])
{
  Vector x(n);
  x(1) = initial_state[0];
  x(2) = initial_state[1];
  x(3) = initial_state[2];
  x(4) = initial_state[3];
  x(5) = initial_state[4];
  Matrix P0(n, n, P0_ini);
  init(x, P0);
}

void EKalmanFilter::setPeriod(double per)
{

  Period = 0.01667; /// Defaults to 60 hz
  Gravity = 9.81;
  period2 = Period * Period;
  period3 = period2 * Period;
  period4 = period3 * Period;
  period5 = period4 * Period;
  period6 = period5 * Period;
}

void EKalmanFilter::makeBaseA()
{
  A(1, 1) = 1.0;
  A(1, 2) = Period;
  A(1, 3) = period2 / 2;
  A(1, 4) = 0.0;
  A(1, 5) = 0.0;

  A(2, 1) = 0.0;
  A(2, 2) = 1.0;
  A(2, 3) = Period;
  A(2, 4) = 0.0;
  A(2, 5) = 0.0;

  A(3, 1) = 0.0;
  A(3, 2) = 0.0;
  A(3, 3) = 1.0;
  A(3, 4) = 0.0;
  A(3, 5) = 0.0;

  A(4, 1) = 0.0;
  A(4, 2) = 0.0;
  A(4, 3) = 0.0;
  A(4, 4) = 1.0;
  A(4, 5) = Period;

  A(5, 1) = 0.0;
  A(5, 2) = 0.0;
  A(5, 3) = 0.0;
  A(5, 4) = 0.0;
  A(5, 5) = 1.0;

}

/// Process noise vector
void EKalmanFilter::makeBaseW()
{
  W(1, 1) = 1.0;
  W(1, 2) = 0.0;
  W(1, 3) = 0.0;
  W(1, 4) = 0.0;
  W(1, 5) = 0.0;

  W(2, 1) = 0.0;
  W(2, 2) = 1.0;
  W(2, 3) = 0.0;
  W(2, 4) = 0.0;
  W(2, 5) = 0.0;

  W(3, 1) = 0.0;
  W(3, 2) = 0.0;
  W(3, 3) = 1.0;
  W(3, 4) = 0.0;
  W(3, 5) = 0.0;

  W(4, 1) = 0.0;
  W(4, 2) = 0.0;
  W(4, 3) = 0.0;
  W(4, 4) = 1.0;
  W(4, 5) = 0.0;

  W(5, 1) = 0.0;
  W(5, 2) = 0.0;
  W(5, 3) = 0.0;
  W(5, 4) = 0.0;
  W(5, 5) = 1.0;
}

void EKalmanFilter::makeBaseQ()
{
  Q(1, 1) = 1.0;
  Q(1, 2) = 0.0;
  Q(1, 3) = 0.0;
  Q(1, 4) = 0.0;
  Q(1, 5) = 0.0;

  Q(2, 1) = 0.0;
  Q(2, 2) = 1.0;
  Q(2, 3) = 0.0;
  Q(2, 4) = 0.0;
  Q(2, 5) = 0.0;

  Q(3, 1) = 0.0;
  Q(3, 2) = 0.0;
  Q(3, 3) = 1.0;
  Q(3, 4) = 0.0;
  Q(3, 5) = 0.0;

  Q(4, 1) = 0.0;
  Q(4, 2) = 0.0;
  Q(4, 3) = 0.0;
  Q(4, 4) = 1.0;
  Q(4, 5) = 0.0;

  Q(5, 1) = 0.0;
  Q(5, 2) = 0.0;
  Q(5, 3) = 0.0;
  Q(5, 4) = 0.0;
  Q(5, 5) = 1.0;

}

/// Linear function that maps measurements to state
void EKalmanFilter::makeBaseH()
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

  H(3, 1) = 1.0;
  H(3, 2) = 0.0;
  H(3, 3) = 1.0;
  H(3, 4) = 0.0;
  H(3, 5) = 1.0;
}

/// V is the measurement noise vector
void EKalmanFilter::makeBaseV()
{
  V(1, 1) = 1.0;
  V(1, 2) = 0.0;
  V(1, 3) = 0.0;
  V(2, 1) = 0.0;
  V(2, 2) = 1.0;
  V(2, 3) = 0.0;
  V(3, 1) = 0.0;
  V(3, 2) = 0.0;
  V(3, 3) = 1.0;

}

/// R is the measurement noise covariance matrix
void EKalmanFilter::makeBaseR()
{
  R(1, 1) = 1.00;
  R(1, 2) = 0.0;
  R(1, 3) = 0.0;

  R(2, 1) = 0.0;
  R(2, 2) = 1.00;
  R(2, 3) = 0.0;

  R(3, 1) = 0.0;
  R(3, 2) = 0.0;
  R(3, 3) = 1.00;
}

void EKalmanFilter::makeProcess()
{
  int i, j;
  Vector x__;
  x__.resize(n);

  for (i = 1; i < n + 1; ++i)
  {
    x__(i) = 0.0;

    for (j = 1; j < n + 1; ++j)
      x__(i) += A(i, j) * x(j);
  }
  x.swap(x__);
}

void EKalmanFilter::makeMeasure()
{
  // z = Hx + Vv
  int i, j;

  z.resize(m);
  for (i = 1; i < m + 1; ++i)
  {
    z(i) = 0.0;
    for (j = 1; j < n + 1; ++j)
      z(i) += H(i, j) * x(j);

  }
}

