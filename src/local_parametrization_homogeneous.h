/*
 * \file local_parametrization_homogeneous.h
 *
 *  Created on: 24/02/2016
 *      Author: jsola
 */

#ifndef LOCALPARAMETRIZATIONHOMOGENEOUS_H_
#define LOCALPARAMETRIZATIONHOMOGENEOUS_H_

#include "local_parametrization_base.h"

/**
 * \brief Local parametrization for homogeneous vectors.
 *
 * The update plus(x,dx) is done orthogonally to x, so that it remains in the sphere defined by the norm of x.
 * Quaternion algebra is used to achieve this effect, with
 *
 *   x_plus_dx = dx ** x
 *
 * where ** is the quaternion product.
 *
 * Contrary to the case of quaternions, it is not required that x is a unit homogeneous vector.
 * The updated x_plus_dx has the same norm as x.
 */
class LocalParametrizationHomogeneous : public LocalParametrizationBase
{
    public:
        LocalParametrizationHomogeneous();
        virtual ~LocalParametrizationHomogeneous();

        virtual bool plus(const Eigen::Map<Eigen::VectorXs>& _h,
                          const Eigen::Map<Eigen::VectorXs>& _delta,
                          Eigen::Map<Eigen::VectorXs>& _h_plus_delta) const;
        virtual bool computeJacobian(const Eigen::Map<Eigen::VectorXs>& _h, Eigen::Map<Eigen::MatrixXs>& _jacobian) const;
};

#endif /* LOCALPARAMETRIZATIONHOMOGENEOUS_H_ */
