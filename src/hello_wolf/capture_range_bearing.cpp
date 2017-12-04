/*
 * CaptureRangeBearing2D.cpp
 *
 *  Created on: Nov 30, 2017
 *      Author: jsola
 */

#include "capture_range_bearing.h"

namespace wolf
{

CaptureRangeBearing::CaptureRangeBearing(const TimeStamp& _ts, const SensorBasePtr& _scanner, const Eigen::VectorXi& _ids, const Eigen::VectorXs& _ranges, const Eigen::VectorXs& _bearings) :
        CaptureBase("RANGE BEARING", _ts, _scanner),
        ids_(_ids),
        ranges_(_ranges),
        bearings_(_bearings)
{
    //
}

CaptureRangeBearing::~CaptureRangeBearing()
{
    //
}

} /* namespace wolf */
