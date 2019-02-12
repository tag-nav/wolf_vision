/**
 * \file feature_polyline_2D.h
 *
 *  Created on: May 26, 2016
 *      \author: jvallve
 */

#ifndef FEATURE_POLYLINE_2D_H_
#define FEATURE_POLYLINE_2D_H_

#include "base/feature/feature_base.h"

namespace wolf
{
WOLF_PTR_TYPEDEFS(FeaturePolyline2D);

//class
class FeaturePolyline2D : public FeatureBase
{
    protected:
        Eigen::MatrixXs points_;
        Eigen::MatrixXs points_cov_;
        bool first_defined_;
        bool last_defined_;

    public:
        FeaturePolyline2D(const Eigen::MatrixXs& _points, const Eigen::MatrixXs& _points_cov, const bool& _first_defined, const bool& _last_defined);
        virtual ~FeaturePolyline2D();
        const Eigen::MatrixXs& getPoints() const;
        const Eigen::MatrixXs& getPointsCov() const;
        bool isFirstDefined() const;
        bool isLastDefined() const;
        int getNPoints() const;
};

inline FeaturePolyline2D::FeaturePolyline2D(const Eigen::MatrixXs& _points, const Eigen::MatrixXs& _points_cov, const bool& _first_defined, const bool& _last_defined) :
        FeatureBase("POLYLINE 2D", Eigen::VectorXs(), Eigen::MatrixXs()),
        points_(_points),
        points_cov_(_points_cov),
        first_defined_(_first_defined),
        last_defined_(_last_defined)
{
    assert(points_.rows() == 3 && points_cov_.rows() == 2 && points_cov_.cols() == 2*points_.cols() && "FeaturePolyline2D::FeaturePolyline2D: Bad points or covariance matrix size");
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

inline bool FeaturePolyline2D::isFirstDefined() const
{
    return first_defined_;
}

inline bool FeaturePolyline2D::isLastDefined() const
{
    return last_defined_;
}

inline int FeaturePolyline2D::getNPoints() const
{
    return points_.cols();
}

} /* namespace wolf */

#endif /* FEATURE_POLYLINE_2D_H_ */
