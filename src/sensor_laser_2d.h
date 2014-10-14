/**
 * \file sensor_laser_2D.h
 *
 *  Created on: 06/10/2014
 *     \author: acorominas
 */

#ifndef SENSOR_LASER_2D_H_
#define SENSOR_LASER_2D_H_

#include "sensor_base.h"

class SensorLaser2D : public SensorBase
{
    private:
        unsigned int n_rays_; ///numper of rays per scan [#]
        double aperture_; ///scan aperture in [rad]
        double range_min_; ///minimum range [m]
        double range_max_; ///maximum range [m]

    public:
        /** \brief Constructor with arguments
         * 
         * Constructor with arguments
         * \param _sp sensor 3D pose with respect to vehicle base frame
         * \param _nrays number of rays per scan
         * \param _apert angular aperture [rad]
         * \param _rmin minimum range [m]
         * \param _rmax maximum range [m]
         * 
         **/
        SensorLaser2D(StatePose& _sp, unsigned int _nrays, double _apert, double _rmin, double _rmax);
        
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
        unsigned int nRays() const;
        
        /** \brief Returns aperture_
         * 
         * Returns aperture_
         * 
         **/        
        double aperture() const;

        /** \brief Returns range_min_
         * 
         * Returns range_min_
         * 
         **/        
        double rangeMin() const;

        /** \brief Returns range_max_
         * 
         * Returns range_max_
         * 
         **/        
        double rangeMax() const;
        
};

////////////////////////////////
// IMPLEMENTATION
////////////////////////////////

inline SensorLaser2D::SensorLaser2D(StatePose& _sp, unsigned int _nrays, double _apert, double _rmin, double _rmax) :
        SensorBase(_sp, _intrinsic, BOTTOM), 
        n_rays_(_nrays), 
        aperture_(_apert), 
        range_min_(_rmin),
        range_max_(_rmax)
{
    //
}

inline SensorLaser2D::~SensorLaser2D()
{
    //
}

inline unsigned int SensorLaser2D::nRays() const
{
    return n_rays_;
}

inline double SensorLaser2D::aperture() const
{
    return aperture_;
}

inline double SensorLaser2D::rangeMin() const
{
    return range_min_;
}

inline double SensorLaser2D::rangeMax() const
{
    return range_max_;
}

#endif /*SENSOR_LASER_2D_H_*/
