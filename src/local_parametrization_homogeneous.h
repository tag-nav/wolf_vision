/*
 * \file local_parametrization_homogeneous.h
 *
 *  Created on: 24/02/2016
 *      Author: jsola
 */

#ifndef LOCALPARAMETRIZATIONHOMOGENEOUS_H_
#define LOCALPARAMETRIZATIONHOMOGENEOUS_H_

#include "local_parametrization_base.h"


namespace wolf {

/**
 * \brief Local parametrization for homogeneous vectors.
 *
 * The composition x_plus_dx = plus(x,dx), written in math mode as \f${\bf x}^+={\bf x}\oplus d {\bf x}\f$,
 * is done orthogonally to \f${\bf x}\f$,
 * so that the result remains in the 4-sphere defined by the norm of \f${\bf x}\f$.
 * Because this is exactly what we do with quaternions,
 * we use quaternion algebra to achieve this effect, with
 *
 *     \f[{\bf x}^+ = {\bf q}( d {\bf x}) \otimes {\bf x}\f]
 *
 * where \f$\otimes\f$ is the quaternion product, and \f$d{\bf q} = {\bf q}(d {\bf x})\f$ is a unit delta_quaternion
 * equivalent to a rotation \f$d{\bf x}\f$, obtained with
 *
 *   \f[{\bf q}(d{\bf x}) = \left[\begin{array}{c} \frac{ d {\bf x}}{ |d{\bf x}|} \sin(|d{\bf x}|)  \\ \cos(|d{\bf x}|) \end{array}\right]\f]
 *
 * Contrary to the case of quaternions, it is not required that \f${\bf x}\f$ be a unit homogeneous vector.
 * In this case, the updated \f${\bf x}^+={\bf x}\oplus d {\bf x}\f$ has the same norm as \f${\bf x}\f$.
 * It is however a good practice to have unit or close-to-unit
 * homogeneous vectors for the sake of numerical stability.
 *
 */
class LocalParametrizationHomogeneous : public LocalParametrizationBase
{
    public:
        LocalParametrizationHomogeneous();
        virtual ~LocalParametrizationHomogeneous();

        virtual bool plus(const Eigen::Map<const Eigen::VectorXs>& _h,
                          const Eigen::Map<const Eigen::VectorXs>& _delta,
                          Eigen::Map<Eigen::VectorXs>& _h_plus_delta) const;
        virtual bool computeJacobian(const Eigen::Map<const Eigen::VectorXs>& _h, Eigen::Map<Eigen::MatrixXs>& _jacobian) const;
};

} // namespace wolf

#endif /* LOCALPARAMETRIZATIONHOMOGENEOUS_H_ */
