/*
 * FeatureRangeBearing2D.h
 *
 *  Created on: Nov 30, 2017
 *      Author: jsola
 */

#ifndef HELLO_WOLF_FEATURE_RANGE_BEARING_H_
#define HELLO_WOLF_FEATURE_RANGE_BEARING_H_

#include "feature_base.h"

namespace wolf
{

WOLF_PTR_TYPEDEFS(FeatureRangeBearing);

class FeatureRangeBearing : public wolf::FeatureBase
{
    public:
        FeatureRangeBearing(const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_covariance);
        virtual ~FeatureRangeBearing();
};

} // namespace wolf

#endif /* HELLO_WOLF_FEATURE_RANGE_BEARING_H_ */
