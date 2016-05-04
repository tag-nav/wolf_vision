#include "sensor_laser_2D.h"
#include "state_block.h"

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
    setType("LASER 2D");
    setDefaultScanParams();
    setDefaultCornerAlgParams();  
}

// SensorLaser2D::SensorLaser2D(const Eigen::VectorXs & _sp, const laserscanutils::ScanParams & _params) :
//     SensorBase(SEN_LIDAR, _sp, 8)
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
    
    scan_params_.angle_min_ = M_PI/2;
    scan_params_.angle_max_ = -M_PI/2;
    scan_params_.angle_step_ = -M_PI/720;
    scan_params_.scan_time_ = 0.01;//not relevant
    scan_params_.range_min_ = 0.2;
    scan_params_.range_max_ = 100;
    scan_params_.range_std_dev_ = 0.01;
    
    setNoise(Eigen::VectorXs::Constant(1,scan_params_.range_std_dev_));


//    params_ << scan_params_.angle_min_, scan_params_.angle_max_, scan_params_.angle_step_,
//               scan_params_.range_min_, scan_params_.range_max_,
//               scan_params_.range_std_dev_,
//               0, scan_params_.scan_time_;
}

void SensorLaser2D::setScanParams(const laserscanutils::ScanParams & _params)
{
    //TODO: Decide who holds intrinsic parameters, either SensorBase::params_ or scan_params_, but NOTH BOTH!!
        
    scan_params_ = _params;
    
//    params_ << scan_params_.angle_min_, scan_params_.angle_max_, scan_params_.angle_step_,
//               scan_params_.range_min_, scan_params_.range_max_,
//               scan_params_.range_std_dev_,
//               0, scan_params_.scan_time_;
}

void SensorLaser2D::setDefaultCornerAlgParams()
{
    //default parameters for corner extraction. TODO: get them from a file or static constants
    corners_alg_params_.theta_min_ = 0.4;
    corners_alg_params_.max_distance_ = 1;
    corners_alg_params_.line_params_.jump_dist_ut_ = 1;
    corners_alg_params_.line_params_.window_length_ = 0.5;
    corners_alg_params_.line_params_.min_window_points_ = 5;
    corners_alg_params_.line_params_.k_sigmas_ut_ = 3; 
    corners_alg_params_.line_params_.concatenate_angle_ut_ = 0.1; 
    corners_alg_params_.line_params_.concatenate_ii_ut_ = 5;
}

void SensorLaser2D::setCornerAlgParams(const laserscanutils::ExtractCornerParams & _corner_alg_params)
{
  corners_alg_params_ = _corner_alg_params;
}

const laserscanutils::ScanParams & SensorLaser2D::getScanParams() const
{
    return scan_params_;
}

const laserscanutils::ExtractCornerParams & SensorLaser2D::getCornerAlgParams() const
{
    return corners_alg_params_;
}

/*Scalar SensorLaser2D::getAngleMin() const
{
    return params_(0);
}

Scalar SensorLaser2D::getAngleMax() const
{
    return params_(1);
}

Scalar SensorLaser2D::getAngleIncrement() const
{
    return params_(2);
}

Scalar SensorLaser2D::getRangeMin() const
{
    return params_(3);
}

Scalar SensorLaser2D::getRangeMax() const
{
    return params_(4);
}

Scalar SensorLaser2D::getRangeStdDev() const
{
    return params_(5);
}

Scalar SensorLaser2D::getTimeIncrement() const
{
    return params_(6);
}

Scalar SensorLaser2D::getScanTime() const
{
    return params_(7);
}*/

void SensorLaser2D::printSensorParameters() const
{
    std::cout << "LASER 2D SENSOR" << std::endl;
    scan_params_.print();
    corners_alg_params_.print();
//     std::cout << "   angle min: " << getAngleMin() << std::endl;
//     std::cout << "   angle min: " << getAngleMax() << std::endl;
//     std::cout << "   angle increment: " << getAngleIncrement() << std::endl;
//     std::cout << "   range min: " << getRangeMin() << std::endl;
//     std::cout << "   range max: " << getRangeMax() << std::endl;
//     std::cout << "   range std dev: " << getRangeStdDev() << std::endl;
//     std::cout << "   time increment: " << getTimeIncrement() << std::endl;
//     std::cout << "   scan time: " << getScanTime() << std::endl;
}

// Define the factory method
SensorBase* SensorLaser2D::create(const std::string& _unique_name, const Eigen::VectorXs& _extrinsics_po,
                                  const IntrinsicsBase* _intrinsics)
{
    // decode extrinsics vector
    assert(_extrinsics_po.size() == 3 && "Bad extrinsics vector length. Should be 3 for 2D.");
    StateBlock* pos_ptr = new StateBlock(_extrinsics_po.head(2), true);
    StateBlock* ori_ptr = new StateBlock(_extrinsics_po.tail(1), true);
    // cast intrinsics into derived type
    IntrinsicsLaser2D* params = (IntrinsicsLaser2D*)(_intrinsics);
    SensorLaser2D* sen = new SensorLaser2D(pos_ptr, ori_ptr);
    sen->setName(_unique_name);
    sen->setScanParams(params->scan_params);
    sen->setCornerAlgParams(params->corners_alg_params);
    return sen;
}

} // namespace wolf
