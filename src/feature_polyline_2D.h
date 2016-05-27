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
        Eigen::MatrixXs points_cov_;
        bool first_extreme_;
        bool last_extreme_;

    public:
        FeaturePolyline2D(const Eigen::MatrixXs& _points, const Eigen::MatrixXs& _points_cov, const bool& _first_extreme, const bool& _last_extreme);
        virtual ~FeaturePolyline2D();
        const Eigen::MatrixXs& getPoints() const;
        const Eigen::MatrixXs& getPointsCov() const;
        bool isFirstExtreme() const;
        bool isLastExtreme() const;
        unsigned int getNPoints() const;
};

inline FeaturePolyline2D::FeaturePolyline2D(const Eigen::MatrixXs& _points, const Eigen::MatrixXs& _points_cov, const bool& _first_extreme, const bool& _last_extreme) :
        FeatureBase(FEATURE_POLYLINE_2D, _points.cols()), points_(_points), points_cov_(_points_cov), first_extreme_(_first_extreme), last_extreme_(_last_extreme)
{
    assert(points_cov_.cols() == 2*points_.cols() && points_cov_.rows() == points_.rows() && "FeaturePolyline2D::FeaturePolyline2D: Bad points or covariance matrix size");
}

inline FeaturePolyline2D::~FeaturePolyline2D()
{
    //
}

inline const Eigen::MatrixXs& FeaturePolyline2D::getPoints() const
{
    return points_;
}

inline const Eigen::MatrixXs& FeaturePolyline2D::getPointsCov() const
{
    return points_cov_;
}

inline bool FeaturePolyline2D::isFirstExtreme() const
{
    return first_extreme_;
}

inline bool FeaturePolyline2D::isLastExtreme() const
{
    return last_extreme_;
}

inline unsigned int FeaturePolyline2D::getNPoints() const
{
    return points_.cols();
}

} /* namespace wolf */

#endif /* FEATURE_POLYLINE_2D_H_ */
