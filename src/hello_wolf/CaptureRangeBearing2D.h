/*
 * CaptureRangeBearing2D.h
 *
 *  Created on: Nov 30, 2017
 *      Author: jsola
 */

#ifndef HELLO_WOLF_CAPTURERANGEBEARING2D_H_
#define HELLO_WOLF_CAPTURERANGEBEARING2D_H_

#include "capture_base.h"

namespace wolf
{

using namespace Eigen;

class CaptureRangeBearing2D : public CaptureBase
{
    public:
        CaptureRangeBearing2D(const TimeStamp& _ts, const SensorBasePtr& _scanner, const Eigen::VectorXs& _ranges, unsigned int _num_points = 271, Scalar _d_angle = 1.0);
        virtual ~CaptureRangeBearing2D();

        VectorXs getRanges();
        VectorXs getBearings();
        Scalar getRange(int _i);
        Scalar getBearing(int _i);
        Vector2s getRangeBearing(int _i);
        Matrix<Scalar, Dynamic, 2> getRangeBearing();

    private:
        VectorXs ranges_;
        VectorXs bearings_;
};

inline Eigen::VectorXs CaptureRangeBearing2D::getRanges()
{
    return ranges_;
}

inline Eigen::VectorXs CaptureRangeBearing2D::getBearings()
{
    return bearings_;
}

inline wolf::Scalar CaptureRangeBearing2D::getRange(int _i)
{
    return ranges_(_i);
}

inline wolf::Scalar CaptureRangeBearing2D::getBearing(int _i)
{
    return bearings_(_i);
}

inline Eigen::Matrix<double,Dynamic,2> CaptureRangeBearing2D::getRangeBearing()
{
    return (Matrix<Scalar,Dynamic,2>() << ranges_, bearings_).finished();
}

inline Eigen::Vector2s CaptureRangeBearing2D::getRangeBearing(int _i)
{
    return Vector2s(ranges_(_i), bearings_(_i));
}

} /* namespace wolf */

#endif /* HELLO_WOLF_CAPTURERANGEBEARING2D_H_ */
