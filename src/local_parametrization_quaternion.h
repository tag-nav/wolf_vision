/*
 * \file local_parametrization_quaternion.h
 *
 *  Created on: Feb 18, 2016
 *      \author: jsola
 */

#ifndef LOCAL_PARAMETRIZATION_QUATERNION_H_
#define LOCAL_PARAMETRIZATION_QUATERNION_H_

#include "local_parametrization_base.h"

/**
 * \brief Local parametrization for quaternions
 *
 * This class implements two possible local parametrizations for quaternions:
 *
 *  - With a local delta,  so that q_new = q_old x q(delta_theta).
 *  - With a global delta, so that q_new = q(delta_theta) x q_old.
 *
 * The choice is selected at construction time.
 */
class LocalParametrizationQuaternion : public LocalParametrizationBase
{
    private:
        bool global_delta_;
    public:
        LocalParametrizationQuaternion(bool _delta_theta_in_global_frame = true);
        virtual ~LocalParametrizationQuaternion();

        virtual bool plus(const Eigen::Map<Eigen::VectorXs>& _q,
                          const Eigen::Map<Eigen::VectorXs>& _delta_theta,
                          Eigen::Map<Eigen::VectorXs>& _q_plus_delta_theta) const;
        virtual bool computeJacobian(const Eigen::Map<Eigen::VectorXs>& _q, Eigen::Map<Eigen::MatrixXs>& _jacobian) const;
};
#endif /* LOCAL_PARAMETRIZATION_QUATERNION_H_ */
