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
 * Local parametrization for homogeneous vectors.
 *
 * The composition plus(x,dx) is done orthogonally to x,
 * so that the result remains in the 4-sphere defined by the norm of x.
 * Because this is exactly what we do with quaternions,
 * we use quaternion algebra to achieve this effect, with
 *
 *   x_plus_dx = q(dx) ** x
 *
 * where ** is the quaternion product, and q(.) is a unit quaternion
 * equivalent to a rotation dx, obtained with
 *
 *   q(dx) = [ dx/|dx|*sin(|dx|) ; cos(|dx|) ]
 *
 * Contrary to the case of quaternions, it is not required that x be a unit homogeneous vector.
 * In this case, the updated x_plus_dx has the same norm as x.
 *
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
