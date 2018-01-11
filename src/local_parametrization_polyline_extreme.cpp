#include "local_parametrization_polyline_extreme.h"
#include "state_block.h"
#include "rotations.h"

namespace wolf {

LocalParametrizationPolylineExtreme::LocalParametrizationPolylineExtreme(StateBlockPtr _reference_point) :
        LocalParametrizationBase(2, 1),
        reference_point_(_reference_point)
{
}

LocalParametrizationPolylineExtreme::~LocalParametrizationPolylineExtreme()
{
}

bool LocalParametrizationPolylineExtreme::plus(Eigen::Map<const Eigen::VectorXs>& _point,
                                               Eigen::Map<const Eigen::VectorXs>& _delta_theta,
                                               Eigen::Map<Eigen::VectorXs>& _point_plus_delta_theta) const
{

    assert(_point.size() == global_size_ && "Wrong size of input point.");
    assert(_delta_theta.size() == local_size_ && "Wrong size of input delta_theta.");
    assert(_point_plus_delta_theta.size() == global_size_ && "Wrong size of output quaternion.");

    _point_plus_delta_theta = reference_point_->getState().head(2) + Eigen::Rotation2Ds(_delta_theta(0)) * (_point - reference_point_->getState().head(2));

    return true;
}

bool LocalParametrizationPolylineExtreme::computeJacobian(Eigen::Map<const Eigen::VectorXs>& _point,
                                                     Eigen::Map<Eigen::MatrixXs>& _jacobian) const
{
    assert(_point.size() == global_size_ && "Wrong size of input point.");
    assert(_jacobian.rows() == global_size_ && _jacobian.cols() == local_size_ && "Wrong size of Jacobian matrix.");

    _jacobian(0,0) =  reference_point_->getState()(1) - _point(1);
    _jacobian(1,0) =  _point(0) - reference_point_->getState()(0);

    return true;
}

bool LocalParametrizationPolylineExtreme::minus(Eigen::Map<const Eigen::VectorXs>& _point1,
                                                Eigen::Map<const Eigen::VectorXs>& _point2,
                                                Eigen::Map<Eigen::VectorXs>& _delta_theta)
{
    Eigen::Vector2s v1 = _point1 - reference_point_->getState().head(2);
    Eigen::Vector2s v2 = _point2 - reference_point_->getState().head(2);

    _delta_theta(0) = pi2pi(atan2(v2(1),v2(0)) - atan2(v1(1),v1(0)));

    return true;
}

} // namespace wolf

