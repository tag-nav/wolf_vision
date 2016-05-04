// sensor_laser_2D.h

#ifndef SENSOR_LASER_2D_H_
#define SENSOR_LASER_2D_H_

//wolf
#include "sensor_base.h"

//laser_scan_utils
#include "laser_scan_utils/scan_basics.h"
#include "laser_scan_utils/corner_detector.h"


namespace wolf {

struct IntrinsicsLaser2D : public IntrinsicsBase
{
        laserscanutils::ScanParams scan_params;
        laserscanutils::ExtractCornerParams corners_alg_params; //parameters for corner extraction algorithm.
};


class SensorLaser2D : public SensorBase
{
    protected:
        laserscanutils::ScanParams scan_params_;
        laserscanutils::ExtractCornerParams corners_alg_params_; //parameters for corner extraction algorithm.

    public:
        /** \brief Constructor with arguments
         * 
         * \param _p_ptr StateBlock pointer to the sensor position
         * \param _o_ptr StateBlock pointer to the sensor orientation
         * 
         **/        
        SensorLaser2D(StateBlock* _p_ptr, StateBlock* _o_ptr);
        //SensorLaser2D(const Eigen::VectorXs & _sp, const laserscanutils::ScanParams & _params);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         * 
         **/
        virtual ~SensorLaser2D();
        
        void setDefaultScanParams();
        
        /** \brief Set scanner intrinsic parameters
         * 
         * \param _params struct with scanner intrinsic parameters. See laser_scan_utils library API for reference.
         * 
         **/                
        void setScanParams(const laserscanutils::ScanParams & _params);
        
        void setDefaultCornerAlgParams();

        /** \brief Set corner detection algorithm parameters
         *
         * Set corner detection algorithm parameters
         * \param _corner_alg_params struct with corner detection algorithm parameters. See laser_scan_utils library API for reference.
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
//         Scalar getAngleMin() const;

        /** \brief Returns angle_max
         *
         * Returns angle_max
         *
         **/
//         Scalar getAngleMax() const;

        /** \brief Returns angle_increment
         *
         * Returns angle_increment
         *
         **/
//         Scalar getAngleIncrement() const;

        /** \brief Returns range_min
         *
         * Returns range_min
         *
         **/
//         Scalar getRangeMin() const;

        /** \brief Returns range_max
         *
         * Returns range_max
         *
         **/
//         Scalar getRangeMax() const;
        
        /** \brief Returns _range_stdev
         * 
         * Returns _range_stdev
         * 
         **/        
//         Scalar getRangeStdDev() const;

        /** \brief Returns time_increment
         *
         * Returns time_increment
         *
         **/
//         Scalar getTimeIncrement() const;

        /** \brief Returns scan_time
         *
         * Returns scan_time
         *
         **/
//         Scalar getScanTime() const;

        /** \brief Prints parameters
         **/                
        void printSensorParameters() const;

    public:
        static SensorBase* create(const std::string& _unique_name, const Eigen::VectorXs& _extrinsics_po, const IntrinsicsBase* _intrinsics);

};

} // namespace wolf


// Register in the SensorFactory
#include "sensor_factory.h"
namespace wolf {
namespace
{
const bool registered_laser = SensorFactory::get()->registerCreator("LASER 2D", SensorLaser2D::create);
}
} // namespace wolf


#endif /*SENSOR_LASER_2D_H_*/
