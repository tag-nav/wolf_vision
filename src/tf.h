/*
 * TF.h
 *
 *  Created on: Nov 23, 2015
 *      Author: jsola
 */

#ifndef SRC_TF_H_
#define SRC_TF_H_

#include <eigen3/Eigen/Dense>

#include "wolf.h"

class TF
{
private:
	Eigen::Vector3s T_;
	Eigen::Matrix3s R_;
	Eigen::Matrix4s H_;
	Eigen::Vector3s T_old_;
	Eigen::Vector4s Q_old_;

	TF() : T_(Eigen::Vector3s::Zero()),
		   R_(Eigen::Matrix3s::Identity()),
		   H_(Eigen::Matrix4s::Identity()),
		   T_old_(T_),
		   Q_old_(Eigen::Vector4s::Zero())
	{
	}

	TF(double* _t, double* _q) : T_(*_t), T_old_(*_t), Q_old_(*_q)
	{
		R_ = Eigen::Quaternion(_q[3],_q[0],_q[1],_q[2]).matrix();
		H_.block<3,3>(0,0) = R_;
		H_.block<3,1>(0,3) = T_;
		H_.block<1,3>(3,0) = Eigen::RowVector3s::Zero();
		H_(3,3) = 1.0;
	}

public:
	void update(double* _t, double* _q)
	{
		if (!equalQ(_t))
		{
			R_ = Eigen::Quaternion(_q[3],_q[0],_q[1],_q[2]).matrix();
			H_.block<3,3>(0,0) = R_;
			H_.block<3,1>(0,3) = T_;
			H_.block<1,3>(3,0) = Eigen::RowVector3s::Zero();
			H_(3,3) = 1.0;
		}
	}


private:
	bool equalT(double * _t)
	{
		return ((_t[0] == T_old_(0)) && (_t[1] == T_old_(1)) && (_t[2] == T_old_(2)));
	}
	bool equalQ(double * _q)
	{
		return ((_q[0] == Q_old_(0)) && (_q[1] == Q_old_(1)) && (_q[2] == Q_old_(2)) && (_q[3] == Q_old_(3)));
	}

};

#endif /* SRC_TF_H_ */
