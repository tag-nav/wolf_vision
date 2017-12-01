/*
 * CaptureRangeBearing2D.cpp
 *
 *  Created on: Nov 30, 2017
 *      Author: jsola
 */

#include "capture_range_bearing.h"

namespace wolf
{

CaptureRangeBearing::CaptureRangeBearing(const TimeStamp& _ts, const SensorBasePtr& _scanner, const Eigen::VectorXs& _ranges, unsigned int _num_points = 271, Scalar _d_angle = 1.0) :
        CaptureBase("RANGE BEARING 2D", _ts, _scanner)
{
    // TODO Auto-generated constructor stub

}

CaptureRangeBearing::~CaptureRangeBearing()
{
    // TODO Auto-generated destructor stub
}

} /* namespace wolf */
