#include "sensor_laser_2D.h"

// SensorLaser2D::SensorLaser2D(const Eigen::VectorXs & _sp, WolfScalar _angle_min, WolfScalar _angle_max, WolfScalar _angle_increment, WolfScalar _range_min, WolfScalar _range_max, WolfScalar _range_stdev, WolfScalar _time_increment, WolfScalar _scan_time) :
//         SensorBase(LIDAR, _sp, 8)
// //        SensorBase(LIDAR, _sp,{(WolfScalar)(_nrays), _apert, _rmin, _rmax, _range_stdev})
// {
//     params_ << _angle_min, _angle_max, _angle_increment, _range_min, _range_max, _range_stdev, _time_increment, _scan_time;
// }

SensorLaser2D::SensorLaser2D(const Eigen::VectorXs & _sp) :
    SensorBase(LIDAR, _sp, 8)
{
    setDefaultScanParams();
    setDefaultCornerAlgParams();  
}

// SensorLaser2D::SensorLaser2D(const Eigen::VectorXs & _sp, const laserscanutils::ScanParams & _params) :
//     SensorBase(LIDAR, _sp, 8)
// {
//     setScanParams(_params);
//     setDefaultCornerAlgParams();  
// }

SensorLaser2D::~SensorLaser2D()
{
    //
}

void SensorLaser2D::setDefaultScanParams()
{
    //TODO: Decide who holds intrinsic parameters, either SensorBase::params_ or scan_params_, but NOTH BOTH!!
    
    scan_params_.angle_min_ = -M_PI/2;
    scan_params_.angle_max_ = M_PI/2;
    scan_params_.angle_step_ = M_PI/720;
    scan_params_.scan_time_ = 0.01;//not relevant
    scan_params_.range_min_ = 0.2;
    scan_params_.range_max_ = 100;
    scan_params_.range_std_dev_ = 0.01;
    
    params_ << scan_params_.angle_min_, scan_params_.angle_max_, scan_params_.angle_step_, 
               scan_params_.range_min_, scan_params_.range_max_, 
               scan_params_.range_std_dev_, 
               0, scan_params_.scan_time_;     
}

void SensorLaser2D::setScanParams(const laserscanutils::ScanParams & _params)
{
    //TODO: Decide who holds intrinsic parameters, either SensorBase::params_ or scan_params_, but NOTH BOTH!!
        
    scan_params_ = _params;
    
    params_ << scan_params_.angle_min_, scan_params_.angle_max_, scan_params_.angle_step_, 
               scan_params_.range_min_, scan_params_.range_max_, 
               scan_params_.range_std_dev_, 
               0, scan_params_.scan_time_;     
}

void SensorLaser2D::setDefaultCornerAlgParams()
{
    //default parameters for corner extraction. TODO: get them from a file or static constants
    corners_alg_params_.segment_window_size = 8;
    corners_alg_params_.theta_min = 0.4;
    corners_alg_params_.theta_max_parallel = 0.1;
    corners_alg_params_.k_sigmas = 3;
    corners_alg_params_.max_beam_distance = 5;
    corners_alg_params_.max_distance = 0.5;

}

const laserscanutils::ScanParams & SensorLaser2D::getScanParams() const
{
    return scan_params_;
}

const laserscanutils::ExtractCornerParams & SensorLaser2D::getCornerAlgParams() const
{
    return corners_alg_params_;
}

/*WolfScalar SensorLaser2D::getAngleMin() const
{
    return params_(0);
}

WolfScalar SensorLaser2D::getAngleMax() const
{
    return params_(1);
}

WolfScalar SensorLaser2D::getAngleIncrement() const
{
    return params_(2);
}

WolfScalar SensorLaser2D::getRangeMin() const
{
    return params_(3);
}

WolfScalar SensorLaser2D::getRangeMax() const
{
    return params_(4);
}

WolfScalar SensorLaser2D::getRangeStdDev() const
{
    return params_(5);
}

WolfScalar SensorLaser2D::getTimeIncrement() const
{
    return params_(6);
}

WolfScalar SensorLaser2D::getScanTime() const
{
    return params_(7);
}*/

void SensorLaser2D::printSensorParameters() const
{
    std::cout << "LASER 2D SENSOR" << std::endl;
//     std::cout << "   angle min: " << getAngleMin() << std::endl;
//     std::cout << "   angle min: " << getAngleMax() << std::endl;
//     std::cout << "   angle increment: " << getAngleIncrement() << std::endl;
//     std::cout << "   range min: " << getRangeMin() << std::endl;
//     std::cout << "   range max: " << getRangeMax() << std::endl;
//     std::cout << "   range std dev: " << getRangeStdDev() << std::endl;
//     std::cout << "   time increment: " << getTimeIncrement() << std::endl;
//     std::cout << "   scan time: " << getScanTime() << std::endl;
}
