/*
 * \file local_parametrization_quaternion.h
 *
 *  Created on: Feb 18, 2016
 *      \author: jsola
 */

#ifndef LOCAL_PARAMETRIZATION_QUATERNION_H_
#define LOCAL_PARAMETRIZATION_QUATERNION_H_

#include "local_parametrization_base.h"


namespace wolf {

/**
  * \brief Local or global orientation error
  *
  * Local or global orientation error.
  *
  * This enum controls whether the orientation error is composed locally or globally to an orientation specification.
  *
  * See LocalParametrizationQuaternion for more information.
  */
typedef enum {
    DQ_LOCAL,
    DQ_GLOBAL
} QuaternionDeltaReference;

/**
 * \brief Local parametrization for quaternions
 *
 * This class implements two possible local parametrizations for quaternions:
 *
 *  - With a local delta,  so that \f${\bf q}^+ = {\bf q} \otimes {\bf q}(d\theta)\f$.
 *  - With a global delta, so that \f${\bf q}^+ = {\bf q}(d\theta) \otimes {\bf q}\f$.
 *
 * The choice is templated, through an enum QuaternionDeltaReference.
 *
 * In either case, the incremental quaternion \f${\bf q}(d\theta)\f$ is computed from the delta_theta
 * variable, here named \f$d\theta\f$, using
 *
 *   \f[{\bf q}(d\theta) =
 *      \left[\begin{array}{c}
 *          \frac{d\theta}{|d\theta|} \sin(|d\theta|) \\
 *          \cos(|d\theta|)
 *      \end{array}\right]
 *   \f]
 *
 */
template <QuaternionDeltaReference DeltaReference = DQ_LOCAL>
class LocalParametrizationQuaternion : public LocalParametrizationBase
{

public:

  LocalParametrizationQuaternion() :
    LocalParametrizationBase(4, 3)
  {
    //
  }

  virtual ~LocalParametrizationQuaternion()
  {
    //
  }

  virtual bool plus(Eigen::Map<const Eigen::VectorXs>& _q,
                    Eigen::Map<const Eigen::VectorXs>& _delta_theta,
                    Eigen::Map<Eigen::VectorXs>& _q_plus_delta_theta) const;

  virtual bool computeJacobian(Eigen::Map<const Eigen::VectorXs>& _q,
                               Eigen::Map<Eigen::MatrixXs>& _jacobian) const;
  virtual bool minus(Eigen::Map<const Eigen::VectorXs>& _q1,
                     Eigen::Map<const Eigen::VectorXs>& _q2,
                     Eigen::Map<Eigen::VectorXs>& _q2_minus_q1);
};

typedef LocalParametrizationQuaternion<DQ_GLOBAL> LocalParametrizationQuaternionGlobal;
typedef LocalParametrizationQuaternion<DQ_LOCAL>  LocalParametrizationQuaternionLocal;

} // namespace wolf

#endif /* LOCAL_PARAMETRIZATION_QUATERNION_H_ */
