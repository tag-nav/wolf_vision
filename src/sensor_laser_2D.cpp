#include "sensor_laser_2D.h"

SensorLaser2D::SensorLaser2D(const Eigen::VectorXs & _sp, WolfScalar _angle_min, WolfScalar _angle_max, WolfScalar _angle_increment, WolfScalar _range_min, WolfScalar _range_max, WolfScalar _range_stdev, WolfScalar _time_increment, WolfScalar _scan_time) :
        SensorBase(LIDAR, _sp, 8)
//        SensorBase(LIDAR, _sp,{(WolfScalar)(_nrays), _apert, _rmin, _rmax, _range_stdev})
{
    params_ << _angle_min, _angle_max, _angle_increment, _range_min, _range_max, _range_stdev, _time_increment, _scan_time;
}

SensorLaser2D::~SensorLaser2D()
{
    //
}

WolfScalar SensorLaser2D::getAngleMin() const
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
}

void SensorLaser2D::printSensorParameters() const
{
    std::cout << "LASER 2D SENSOR" << std::endl;
    std::cout << "   angle min: " << getAngleMin() << std::endl;
    std::cout << "   angle min: " << getAngleMax() << std::endl;
    std::cout << "   angle increment: " << getAngleIncrement() << std::endl;
    std::cout << "   range min: " << getRangeMin() << std::endl;
    std::cout << "   range max: " << getRangeMax() << std::endl;
    std::cout << "   range std dev: " << getRangeStdDev() << std::endl;
    std::cout << "   time increment: " << getTimeIncrement() << std::endl;
    std::cout << "   scan time: " << getScanTime() << std::endl;
}
