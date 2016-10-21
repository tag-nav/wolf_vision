#include "local_parametrization_polyline_extreme.h"
#include "state_block.h"

namespace wolf {

LocalParametrizationPolylineExtreme::LocalParametrizationPolylineExtreme(StateBlockPtr _reference_point) :
        LocalParametrizationBase(2, 1),
        reference_point_(_reference_point)
{
}

LocalParametrizationPolylineExtreme::~LocalParametrizationPolylineExtreme()
{
}

bool LocalParametrizationPolylineExtreme::plus(const Eigen::Map<const Eigen::VectorXs>& _point,
                                               const Eigen::Map<const Eigen::VectorXs>& _delta_theta,
                                               Eigen::Map<Eigen::VectorXs>& _point_plus_delta_theta) const
{

    assert(_point.size() == global_size_ && "Wrong size of input point.");
    assert(_delta_theta.size() == local_size_ && "Wrong size of input delta_theta.");
    assert(_point_plus_delta_theta.size() == global_size_ && "Wrong size of output quaternion.");

    _point_plus_delta_theta = reference_point_->getVector().head(2) + Eigen::Rotation2Ds(_delta_theta(0)) * (_point - reference_point_->getVector().head(2));

    return true;
}

bool LocalParametrizationPolylineExtreme::computeJacobian(const Eigen::Map<const Eigen::VectorXs>& _point,
                                                     Eigen::Map<Eigen::MatrixXs>& _jacobian) const
{
    assert(_point.size() == global_size_ && "Wrong size of input point.");
    assert(_jacobian.rows() == global_size_ && _jacobian.cols() == local_size_ && "Wrong size of Jacobian matrix.");

    _jacobian(0,0) =  reference_point_->getVector()(1) - _point(1);
    _jacobian(1,0) =  _point(0) - reference_point_->getVector()(0);

    return true;
}

} // namespace wolf
