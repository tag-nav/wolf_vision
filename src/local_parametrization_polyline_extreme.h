/*
 * \file local_parametrization_polyline_extreme.h
 *
 *  Created on: Jun 10, 2016
 *      \author: jvallve
 */

#ifndef LOCAL_PARAMETRIZATION_POLYLINE_EXTREME_H_
#define LOCAL_PARAMETRIZATION_POLYLINE_EXTREME_H_

#include "local_parametrization_base.h"


namespace wolf {

/**
 * \brief Local parametrization for polylines not defined extreme points
 *
 */
class LocalParametrizationPolylineExtreme : public LocalParametrizationBase
{
    private:
        StateBlockPtr reference_point_;
    public:
        LocalParametrizationPolylineExtreme(StateBlockPtr _reference_point);
        virtual ~LocalParametrizationPolylineExtreme();

        virtual bool plus(Eigen::Map<const Eigen::VectorXs>& _point,
                          Eigen::Map<const Eigen::VectorXs>& _delta_theta,
                          Eigen::Map<Eigen::VectorXs>& _point_plus_delta_theta) const;
        virtual bool computeJacobian(Eigen::Map<const Eigen::VectorXs>& _point, Eigen::Map<Eigen::MatrixXs>& _jacobian) const;
        virtual bool minus(Eigen::Map<const Eigen::VectorXs>& _point1,
                           Eigen::Map<const Eigen::VectorXs>& _point2,
                           Eigen::Map<Eigen::VectorXs>& _delta_theta);
};

} // namespace wolf

#endif /* LOCAL_PARAMETRIZATION_POLYLINE_EXTREME_H_ */
