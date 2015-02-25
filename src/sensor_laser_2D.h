// sensor_laser_2D.h

#ifndef SENSOR_LASER_2D_H_
#define SENSOR_LASER_2D_H_

//wolf includes
#include "sensor_base.h"

class SensorLaser2D : public SensorBase
{
    private:
//         unsigned int n_rays_; ///numper of rays per scan [#]
//         double aperture_; ///scan aperture in [rad]
//         double range_min_; ///minimum range [m]
//         double range_max_; ///maximum range [m]

    public:
        /** \brief Constructor with arguments
         * 
         * Constructor with arguments
         * \param _sp sensor 3D pose with respect to vehicle base frame
         * \param _angle_min start angle of the scan [rad]
         * \param _angle_max end angle of the scan [rad]
         * \param _angle_increment angular distance between measurements [rad]
         * \param _range_min minimum range value [m]
         * \param _range_max maximum range value [m]
         * \param _range_stdev range standard deviation [m]
         * \param _time_increment time between beams [seconds]
         * \param _scan_time time between scans [seconds]
         * 
         **/
        SensorLaser2D(const Eigen::VectorXs & _sp, WolfScalar _angle_min, WolfScalar _angle_max, WolfScalar _angle_increment, WolfScalar _range_min, WolfScalar _range_max, WolfScalar _range_stdev, WolfScalar _time_increment=0, WolfScalar _scan_time=0);

        /** \brief Destructor
         * 
         * Destructor
         * 
         **/
        virtual ~SensorLaser2D();

        /** \brief Returns angle_min
         *
         * Returns angle_min
         *
         **/
        WolfScalar getAngleMin() const;

        /** \brief Returns angle_max
         *
         * Returns angle_max
         *
         **/
        WolfScalar getAngleMax() const;

        /** \brief Returns angle_increment
         *
         * Returns angle_increment
         *
         **/
        WolfScalar getAngleIncrement() const;

        /** \brief Returns range_min
         *
         * Returns range_min
         *
         **/
        WolfScalar getRangeMin() const;

        /** \brief Returns range_max
         *
         * Returns range_max
         *
         **/
        WolfScalar getRangeMax() const;
        
        /** \brief Returns _range_stdev
         * 
         * Returns _range_stdev
         * 
         **/        
        WolfScalar getRangeStdDev() const;

        /** \brief Returns time_increment
         *
         * Returns time_increment
         *
         **/
        WolfScalar getTimeIncrement() const;

        /** \brief Returns scan_time
         *
         * Returns scan_time
         *
         **/
        WolfScalar getScanTime() const;

        /** \brief Prints parameters
         * 
         * Prints parameters
         * 
         **/                
        void printSensorParameters() const;
};
#endif /*SENSOR_LASER_2D_H_*/
