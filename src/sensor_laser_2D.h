// sensor_laser_2D.h

#ifndef SENSOR_LASER_2D_H_
#define SENSOR_LASER_2D_H_

//laser_scan_utils
#include "iri-algorithms/laser_scan_utils/scan_basics.h"
#include "iri-algorithms/laser_scan_utils/corner_detector.h"

//wolf 
#include "sensor_base.h"

class SensorLaser2D : public SensorBase
{
    protected:
//         unsigned int n_rays_; ///numper of rays per scan [#]
//         double aperture_; ///scan aperture in [rad]
//         double range_min_; ///minimum range [m]
//         double range_max_; ///maximum range [m]
        laserscanutils::ScanParams scan_params_;//TODO: Decide who holds intrinsic parameters, either SensorBase::params_ or scan_params_, but NOTH BOTH!!
        laserscanutils::ExtractCornerParams corners_alg_params_; //parameters for corner extraction algorithm.

    public:
        /** \brief Constructor with arguments
         * 
         * Constructor with arguments
         * \param _p_ptr StateBase pointer to the sensor position
         * \param _o_ptr StateOrientation pointer to the sensor orientation
         * \param _params struct with scan parameters. See laser_scan_utils library API for reference
         * 
         **/        
        SensorLaser2D(StatePoint3D* _p_ptr, StateOrientation* _o_ptr);
        //SensorLaser2D(const Eigen::VectorXs & _sp, const laserscanutils::ScanParams & _params);

        /** \brief Destructor
         * 
         * Destructor
         * 
         **/
        virtual ~SensorLaser2D();
        
        void setDefaultScanParams();
        
        /** \brief Set scanner intrinsic parameters
         * 
         * Set scanner intrinsic parameters
         * \param _params struct with scanner intrinsic parameters. See laser_scan_utils library API for reference.
         * 
         **/                
        void setScanParams(const laserscanutils::ScanParams & _params);
        
        void setDefaultCornerAlgParams();

        /** \brief Set corner detection algorithm parameters
         *
         * Set corner detection algorithm parameters
         * \param _params struct with corner detection algorithm parameters. See laser_scan_utils library API for reference.
         *
         **/
        void setCornerAlgParams(const laserscanutils::ExtractCornerParams & _corner_alg_params);

        /** \brief Get scanner intrinsic parameters
         * 
         * Get scanner intrinsic parameters
         * 
         **/                        
        const laserscanutils::ScanParams & getScanParams() const;

        /** \brief Get parameters for corner detection algorithm
         * 
         * Get parameters for corner detection algorithm
         * 
         **/                                
        const laserscanutils::ExtractCornerParams & getCornerAlgParams() const;

        /** \brief Returns angle_min
         *
         * Returns angle_min
         *
         **/
//         WolfScalar getAngleMin() const;

        /** \brief Returns angle_max
         *
         * Returns angle_max
         *
         **/
//         WolfScalar getAngleMax() const;

        /** \brief Returns angle_increment
         *
         * Returns angle_increment
         *
         **/
//         WolfScalar getAngleIncrement() const;

        /** \brief Returns range_min
         *
         * Returns range_min
         *
         **/
//         WolfScalar getRangeMin() const;

        /** \brief Returns range_max
         *
         * Returns range_max
         *
         **/
//         WolfScalar getRangeMax() const;
        
        /** \brief Returns _range_stdev
         * 
         * Returns _range_stdev
         * 
         **/        
//         WolfScalar getRangeStdDev() const;

        /** \brief Returns time_increment
         *
         * Returns time_increment
         *
         **/
//         WolfScalar getTimeIncrement() const;

        /** \brief Returns scan_time
         *
         * Returns scan_time
         *
         **/
//         WolfScalar getScanTime() const;

        /** \brief Prints parameters
         * 
         * Prints parameters
         * 
         **/                
        void printSensorParameters() const;
};
#endif /*SENSOR_LASER_2D_H_*/
