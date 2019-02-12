
#include "base/feature/feature_corner_2D.h"

namespace wolf {

FeatureLine2D::FeatureLine2D(const Eigen::Vector3s & _line_homogeneous_params, 
                             const Eigen::Matrix3s & _params_covariance, Eigen::Vector3s & _point1, Eigen::Vector3s & _point2) :
               FeatureBase("LINE_2D", _line_homogeneous_params, _params_covariance),
               first_point_(_point1),
               last_point_(_point2)
{
    //
}

FeatureLine2D::~FeatureLine2D()
{
    //
}

} // namespace wolf
