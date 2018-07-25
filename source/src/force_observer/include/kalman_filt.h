/******************************************************************************
 * @file    kalman_filt.h
 * @authors  LH
 * @date    2016/06/17
 * @brief   Kalman filter of a simple dynamic system.
 *          This class implements a kalman filter where there is no input to
 *          the system.
 ******************************************************************************/
#ifndef HUMANOID_NAVIGATION_CONTROLLER_WALK_CONTROLLER_INCLUDE_KALMAN_FILT_H_
#define HUMANOID_NAVIGATION_CONTROLLER_WALK_CONTROLLER_INCLUDE_KALMAN_FILT_H_

#include "ekfilter.hpp"

class cPlaneEKF: public Kalman::EKFilter<double, 1, false, false, true>
{
	public:
		cPlaneEKF();
		void setPeriod(double sampling_period);

	protected:
		void makeBaseA();
		void makeBaseH();
		void makeBaseV();
		void makeBaseR();
		void makeBaseW();
		void makeBaseQ();
		void makeProcess();
		void makeMeasure();
		cPlaneEKF::Matrix B;

		double Period, Gravity;
		double period6, period5, period4, period3, period2;
};

typedef cPlaneEKF::Vector Vector;
typedef cPlaneEKF::Matrix Matrix;

#endif /* HUMANOID_NAVIGATION_CONTROLLER_WALK_CONTROLLER_INCLUDE_KALMAN_FILT_H_ */
