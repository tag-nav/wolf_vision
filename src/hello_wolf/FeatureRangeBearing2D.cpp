/*
 * FeatureRangeBearing2D.cpp
 *
 *  Created on: Nov 30, 2017
 *      Author: jsola
 */

#include "FeatureRangeBearing2D.h"

namespace wolf
{

FeatureRangeBearing2D::FeatureRangeBearing2D(const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_covariance) :
        FeatureBase("RANGE BEARING 2D", _measurement, _meas_covariance)
{
    //
}


FeatureRangeBearing2D::~FeatureRangeBearing2D()
{
    // TODO Auto-generated destructor stub
}

} // namespace wolf

