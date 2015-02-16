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
         * \param _nrays number of rays per scan
         * \param _apert angular aperture [rad]
         * \param _rmin minimum range [m]
         * \param _rmax maximum range [m]
         * \param _range_stdev range standard deviation [m]
         * 
         **/
        SensorLaser2D(const Eigen::VectorXs & _sp, unsigned int _nrays, WolfScalar _apert, WolfScalar _rmin, WolfScalar _rmax, WolfScalar _range_stdev);
        
        /** \brief Destructor
         * 
         * Destructor
         * 
         **/
        virtual ~SensorLaser2D();

        /** \brief Returns n_rays_
         * 
         * Returns n_rays_
         * 
         **/        
        unsigned int getNumRays() const;
        
        /** \brief Returns aperture_
         * 
         * Returns aperture_
         * 
         **/        
        double getAperture() const;

        /** \brief Returns range_min_
         * 
         * Returns range_min_
         * 
         **/        
        double getRangeMin() const;

        /** \brief Returns range_max_
         * 
         * Returns range_max_
         * 
         **/        
        double getRangeMax() const;
        
};
#endif /*SENSOR_LASER_2D_H_*/
