#include "sensor_laser_2D.h"

namespace wolf {

// SensorLaser2D::SensorLaser2D(const Eigen::VectorXs & _sp, Scalar _angle_min, Scalar _angle_max, Scalar _angle_increment, Scalar _range_min, Scalar _range_max, Scalar _range_stdev, Scalar _time_increment, Scalar _scan_time) :
//         SensorBase(LIDAR, _sp, 8)
// //        SensorBase(LIDAR, _sp,{(Scalar)(_nrays), _apert, _rmin, _rmax, _range_stdev})
// {
//     params_ << _angle_min, _angle_max, _angle_increment, _range_min, _range_max, _range_stdev, _time_increment, _scan_time;
// }

SensorLaser2D::SensorLaser2D(StateBlock* _p_ptr, StateBlock* _o_ptr) :
    SensorBase(SEN_LIDAR, _p_ptr, _o_ptr, nullptr, 8)
{
    setDefaultScanParams();
}

SensorLaser2D::SensorLaser2D(StateBlock* _p_ptr, StateBlock* _o_ptr, const double& _angle_min, const double& _angle_max, const double& _angle_step, const double& _scan_time, const double& _range_min, const double& _range_max, const double& _range_std_dev, const double& _angle_std_dev) :
        SensorBase(SEN_LIDAR, _p_ptr, _o_ptr, nullptr, 8),
        scan_params_({ _angle_min, _angle_max, _angle_step, _scan_time, _range_min, _range_max, _range_std_dev, _angle_std_dev })
{
}

SensorLaser2D::~SensorLaser2D()
{
    //
}

void SensorLaser2D::setDefaultScanParams()
{
    scan_params_.angle_min_ = M_PI/2;
    scan_params_.angle_max_ = -M_PI/2;
    scan_params_.angle_step_ = -M_PI/720;
    scan_params_.scan_time_ = 0.01;//not relevant
    scan_params_.range_min_ = 0.2;
    scan_params_.range_max_ = 100;
    scan_params_.range_std_dev_ = 0.01;
    
    setNoise(Eigen::VectorXs::Constant(1,scan_params_.range_std_dev_));
}

void SensorLaser2D::setScanParams(const laserscanutils::LaserScanParams & _params)
{
    scan_params_ = _params;
}

const laserscanutils::LaserScanParams& SensorLaser2D::getScanParams() const
{
    return scan_params_;
}

} // namespace wolf
