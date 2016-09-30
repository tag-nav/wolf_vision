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
 * The choice is selected at construction time, through an enum QuaternionDeltaReference.
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
template <unsigned int DeltaReference/* = DQ_GLOBAL*/>
class LocalParametrizationQuaternion : public LocalParametrizationBase
{
protected:

  QuaternionDeltaReference delta_reference_;

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

  virtual bool plus(const Eigen::Map<const Eigen::VectorXs>& _q,
                    const Eigen::Map<const Eigen::VectorXs>& _delta_theta,
                    Eigen::Map<Eigen::VectorXs>& _q_plus_delta_theta) const;

  virtual bool computeJacobian(const Eigen::Map<const Eigen::VectorXs>& _q,
                               Eigen::Map<Eigen::MatrixXs>& _jacobian) const;
};

// @TODO : considere typedef
// typedef LocalParametrizationQuaternion<DQ_GLOBAL> LocalParametrizationQuaternionG;
// typedef LocalParametrizationQuaternion<DQ_LOCAL>  LocalParametrizationQuaternionL;

} // namespace wolf

#endif /* LOCAL_PARAMETRIZATION_QUATERNION_H_ */
