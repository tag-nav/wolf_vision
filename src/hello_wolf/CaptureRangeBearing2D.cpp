/*
 * CaptureRangeBearing2D.cpp
 *
 *  Created on: Nov 30, 2017
 *      Author: jsola
 */

#include "CaptureRangeBearing2D.h"

namespace wolf
{

CaptureRangeBearing2D::CaptureRangeBearing2D(const TimeStamp& _ts, const SensorBasePtr& _scanner, const Eigen::VectorXs& _ranges, unsigned int _num_points = 271, Scalar _d_angle = 1.0) :
        CaptureBase("RANGE BEARING 2D", _ts, _scanner)
{
    // TODO Auto-generated constructor stub

}

CaptureRangeBearing2D::~CaptureRangeBearing2D()
{
    // TODO Auto-generated destructor stub
}

} /* namespace wolf */
