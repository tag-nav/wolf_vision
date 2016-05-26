/**
 * \file feature_polyline_2D.h
 *
 *  Created on: May 26, 2016
 *      \author: jvallve
 */

#ifndef FEATURE_POLYLINE_2D_H_
#define FEATURE_POLYLINE_2D_H_

#include "feature_base.h"

namespace wolf
{

class FeaturePolyline2D : public FeatureBase
{
    protected:
        Eigen::MatrixXs points_;
        bool first_extreme_;
        bool last_extreme_;

    public:
        FeaturePolyline2D(const Eigen::MatrixXs& _points, const bool& _first_extreme, const bool& _last_extreme);
        virtual ~FeaturePolyline2D();
        const Eigen::MatrixXs& getPoints() const;
        bool isFirstExtreme() const;
        bool isLastExtreme() const;
};

inline FeaturePolyline2D::FeaturePolyline2D(const Eigen::MatrixXs& _points, const bool& _first_extreme, const bool& _last_extreme) :
        FeatureBase(FEATURE_POLYLINE_2D, _points.cols()), points_(_points), first_extreme_(_first_extreme), last_extreme_(_last_extreme)
{
}

inline FeaturePolyline2D::~FeaturePolyline2D()
{
    //
}

inline const Eigen::MatrixXs& FeaturePolyline2D::getPoints() const
{
    return points_;
}

inline bool FeaturePolyline2D::isFirstExtreme() const
{
    return first_extreme_;
}

inline bool FeaturePolyline2D::isLastExtreme() const
{
    return last_extreme_;
}


} /* namespace wolf */

#endif /* FEATURE_POLYLINE_2D_H_ */
