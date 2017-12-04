/*
 * capture_range_bearing.h
 *
 *  Created on: Nov 30, 2017
 *      Author: jsola
 */

#ifndef HELLO_WOLF_CAPTURE_RANGE_BEARING_H_
#define HELLO_WOLF_CAPTURE_RANGE_BEARING_H_

#include "capture_base.h"

namespace wolf
{

WOLF_PTR_TYPEDEFS(CaptureRangeBearing)

using namespace Eigen;

class CaptureRangeBearing : public CaptureBase
{
    public:
        CaptureRangeBearing(const TimeStamp& _ts, const SensorBasePtr& _scanner, const Eigen::VectorXi& _ids, const Eigen::VectorXs& _ranges, const Eigen::VectorXs& _bearings);
        virtual ~CaptureRangeBearing();

        const VectorXi&         getIds          ()          const;
        const int&              getId           (int _i)    const;
        const Eigen::VectorXs&  getRanges       ()          const;
        const Eigen::VectorXs&  getBearings     ()          const;
        const wolf::Scalar&     getRange        (int _i)    const;
        const wolf::Scalar&     getBearing      (int _i)    const;
        Eigen::Vector2s         getRangeBearing (int _i)    const;
        Eigen::Matrix<double, Dynamic, 2> getRangeBearing() const;

    private:
        VectorXi ids_;          // identifiers
        VectorXs ranges_;       // ranges
        VectorXs bearings_;     // bearings
};

inline const Eigen::VectorXi& CaptureRangeBearing::getIds() const
{
    return ids_;
}

inline const int& CaptureRangeBearing::getId(int _i) const
{
    return ids_(_i);
}

inline const Eigen::VectorXs& CaptureRangeBearing::getRanges() const
{
    return ranges_;
}

inline const Eigen::VectorXs& CaptureRangeBearing::getBearings() const
{
    return bearings_;
}

inline const wolf::Scalar& CaptureRangeBearing::getRange(int _i) const
{
    return ranges_(_i);
}

inline const wolf::Scalar& CaptureRangeBearing::getBearing(int _i) const
{
    return bearings_(_i);
}

inline Eigen::Matrix<Scalar,Dynamic,2> CaptureRangeBearing::getRangeBearing() const
{
    return (Matrix<Scalar,Dynamic,2>() << ranges_, bearings_).finished();
}

inline Eigen::Vector2s CaptureRangeBearing::getRangeBearing(int _i) const
{
    return Vector2s(ranges_(_i), bearings_(_i));
}

} /* namespace wolf */

#endif /* HELLO_WOLF_CAPTURE_RANGE_BEARING_H_ */
