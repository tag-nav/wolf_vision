/*
 * FeatureRangeBearing2D.h
 *
 *  Created on: Nov 30, 2017
 *      Author: jsola
 */

#ifndef HELLO_WOLF_FEATURERANGEBEARING2D_H_
#define HELLO_WOLF_FEATURERANGEBEARING2D_H_

#include "feature_base.h"

namespace wolf
{

}
class FeatureRangeBearing2D : public wolf::FeatureBase
{
    public:
        FeatureRangeBearing2D(const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_covariance);
        virtual ~FeatureRangeBearing2D();
};

} // namespace wolf

#endif /* HELLO_WOLF_FEATURERANGEBEARING2D_H_ */
