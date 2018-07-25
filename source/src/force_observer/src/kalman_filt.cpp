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

#include "kalman_filt.h"
#include <cmath>
#include <iostream>

using namespace std;

cPlaneEKF::cPlaneEKF()
{
	setDim(3, 1, 3, 1, 1);
	Period = 0.01667; /// Defaults to 60 hz
	Gravity = 9.8;
	period2 = Period * Period;
	period3 = period2 * Period;
	period4 = period3 * Period;
	period5 = period4 * Period;
	period6 = period5 * Period;

}

void cPlaneEKF::setPeriod(double sampling_period)
{

	Period = sampling_period;
	period2 = Period * Period;
	period3 = period2 * Period;
	period4 = period3 * Period;
	period5 = period4 * Period;
	period6 = period5 * Period;

}


void cPlaneEKF::makeBaseA()
{
	A(1, 1) = 1.0;
	A(1, 2) = Period;
	A(1, 3) = Period * Period / 2;

	A(2, 1) = 0.0;
	A(2, 2) = 1.0;
	A(2, 3) = Period;

	A(3, 1) = 0.0;
	A(3, 2) = 0.0;
	A(3, 3) = 1.0;
}

void cPlaneEKF::makeBaseW()
{
	W(1, 1) = 1.0;
	W(1, 2) = 0.0;
	W(1, 3) = 0.0;
	W(2, 1) = 0.0;
	W(2, 2) = 1.0;
	W(2, 3) = 0.0;
	W(3, 1) = 0.0;
	W(3, 2) = 0.0;
	W(3, 3) = 1.0;
}

void cPlaneEKF::makeBaseQ()
{
	int r = 1;
	Q(1, 1) = r * 0.01625 * period6 / 36.0;
	Q(1, 2) = r * 0.01625 * period5 / 12.0;
	Q(1, 3) = r * 0.01625 * period4 / 6.0;
	Q(2, 1) = r * 0.01625 * period5 / 12.0;
	Q(2, 2) = r * 0.01625 * period4 / 6.0;
	Q(2, 3) = r * 0.01625 * period3 / 2.0;
	Q(3, 1) = r * 0.01625 * period4 / 6.0;
	Q(3, 2) = r * 0.01625 * period3 / 2.0;
	Q(3, 3) = r * 0.01625 * period2;
}

void cPlaneEKF::makeBaseH()
{
	H(1, 1) = 0.0;
	H(1, 2) = 0.0;
	H(1, 3) = 1.0;
}

void cPlaneEKF::makeBaseV()
{
	V(1, 1) = 1.0;

}

void cPlaneEKF::makeBaseR()
{
	R(1, 1) = 0.000125;
}

void cPlaneEKF::makeProcess()
{
	x(1) = A(1, 1) * x(1) + A(1, 2) * x(2) + A(1, 3) * x(3);
	x(2) = A(2, 1) * x(1) + A(2, 2) * x(2) + A(2, 3) * x(3);
	x(3) = A(3, 1) * x(1) + A(3, 2) * x(2) + A(3, 3) * x(3);

}

void cPlaneEKF::makeMeasure()
{
	z(1) = x(3);
}

