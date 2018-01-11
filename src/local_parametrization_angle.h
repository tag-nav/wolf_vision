/**
 * \file local_parametrization_angle.h
 *
 *  Created on: Apr 4, 2017
 *      \author: jsola
 */

#ifndef LOCAL_PARAMETRIZATION_ANGLE_H_
#define LOCAL_PARAMETRIZATION_ANGLE_H_

#include "local_parametrization_base.h"
#include "rotations.h"

namespace wolf
{

class LocalParametrizationAngle : public LocalParametrizationBase
{
    public:
        LocalParametrizationAngle();
        virtual ~LocalParametrizationAngle();

        virtual bool plus(Eigen::Map<const Eigen::VectorXs>& _h, Eigen::Map<const Eigen::VectorXs>& _delta,
                          Eigen::Map<Eigen::VectorXs>& _h_plus_delta) const;
        virtual bool computeJacobian(Eigen::Map<const Eigen::VectorXs>& _h,
                                     Eigen::Map<Eigen::MatrixXs>& _jacobian) const;
        virtual bool minus(Eigen::Map<const Eigen::VectorXs>& _x1,
                           Eigen::Map<const Eigen::VectorXs>& _x2,
                           Eigen::Map<Eigen::VectorXs>& _x2_minus_x1);

};

inline LocalParametrizationAngle::LocalParametrizationAngle() :
        LocalParametrizationBase(1,1)
{
    //
}

inline LocalParametrizationAngle::~LocalParametrizationAngle()
{
    //
}

inline bool LocalParametrizationAngle::plus(Eigen::Map<const Eigen::VectorXs>& _h,
                                            Eigen::Map<const Eigen::VectorXs>& _delta,
                                            Eigen::Map<Eigen::VectorXs>& _h_plus_delta) const
{
    _h_plus_delta(0) = pi2pi(_h(0) + _delta(0));
    return true;
}

inline bool LocalParametrizationAngle::computeJacobian(Eigen::Map<const Eigen::VectorXs>& _h,
                                                       Eigen::Map<Eigen::MatrixXs>& _jacobian) const
{
    _jacobian(0) = 1.0;
    return true;
}

inline bool LocalParametrizationAngle::minus(Eigen::Map<const Eigen::VectorXs>& _x1,
                                             Eigen::Map<const Eigen::VectorXs>& _x2,
                                             Eigen::Map<Eigen::VectorXs>& _x2_minus_x1)
{
    _x2_minus_x1(0) = pi2pi(_x2(0)-_x1(0));
    return true;
}

} /* namespace wolf */

#endif /* LOCAL_PARAMETRIZATION_ANGLE_H_ */
