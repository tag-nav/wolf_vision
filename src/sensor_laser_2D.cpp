#include "sensor_laser_2D.h"

SensorLaser2D::SensorLaser2D(const Eigen::VectorXs & _sp, unsigned int _nrays, WolfScalar _apert, WolfScalar _rmin, WolfScalar _rmax, WolfScalar _range_stdev) :
        SensorBase(LIDAR, _sp, 5)
//         SensorBase(LIDAR, _sp,{(WolfScalar)(_nrays), _apert, _rmin, _rmax, _range_stdev})
{
    params_ << (WolfScalar)(_nrays), _apert, _rmin, _rmax, _range_stdev;
}

SensorLaser2D::~SensorLaser2D()
{
    //
}

unsigned int SensorLaser2D::getNumRays() const
{
    return (unsigned int)(params_(0));
}

WolfScalar SensorLaser2D::getAperture() const
{
    return params_(1);
}

WolfScalar SensorLaser2D::getRangeMin() const
{
    return params_(2);
}

WolfScalar SensorLaser2D::getRangeMax() const
{
    return params_(3);
}
