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

        virtual bool plus(const Eigen::Map<const Eigen::VectorXs>& _point,
                          const Eigen::Map<const Eigen::VectorXs>& _delta_theta,
                          Eigen::Map<Eigen::VectorXs>& _point_plus_delta_theta) const;
        virtual bool computeJacobian(const Eigen::Map<const Eigen::VectorXs>& _point, Eigen::Map<Eigen::MatrixXs>& _jacobian) const;
};

} // namespace wolf

#endif /* LOCAL_PARAMETRIZATION_POLYLINE_EXTREME_H_ */
