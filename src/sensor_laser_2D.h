// sensor_laser_2D.h

#ifndef SENSOR_LASER_2D_H_
#define SENSOR_LASER_2D_H_

//wolf
#include "sensor_base.h"

//laser_scan_utils
#include "laser_scan_utils/laser_scan.h"

namespace wolf {

struct IntrinsicsLaser2D : public IntrinsicsBase
{
    laserscanutils::LaserScanParams scan_params;
};


class SensorLaser2D : public SensorBase
{
    protected:
        laserscanutils::LaserScanParams scan_params_;

    public:
        /** \brief Constructor with extrinsics
         * 
         * \param _p_ptr StateBlock pointer to the sensor position
         * \param _o_ptr StateBlock pointer to the sensor orientation
         * 
         **/        
        SensorLaser2D(StateBlock* _p_ptr, StateBlock* _o_ptr);

        /** \brief Constructor with extrinsics and scan parameters
         *
         * \param _p_ptr StateBlock pointer to the sensor position
         * \param _o_ptr StateBlock pointer to the sensor orientation
         *
         **/
        SensorLaser2D(StateBlock* _p_ptr, StateBlock* _o_ptr, const double& _angle_min, const double& _angle_max, const double& _angle_step, const double& _scan_time, const double& _range_min, const double& _range_max, const double& _range_std_dev, const double& _angle_std_dev);

        /** \brief Constructor with extrinsics and scan parameters
         *
         * \param _p_ptr StateBlock pointer to the sensor position
         * \param _o_ptr StateBlock pointer to the sensor orientation
         * \param _params Scan parameters
         *
         **/
        SensorLaser2D(StateBlock* _p_ptr, StateBlock* _o_ptr, const laserscanutils::LaserScanParams& _params);

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
        void setScanParams(const laserscanutils::LaserScanParams & _params);

        /** \brief Get scanner intrinsic parameters
         * 
         * Get scanner intrinsic parameters
         * 
         **/                        
        const laserscanutils::LaserScanParams & getScanParams() const;

    public:
        static SensorBase* create(const std::string& _unique_name, const Eigen::VectorXs& _extrinsics_po, const IntrinsicsBase* _intrinsics);

};

} // namespace wolf

#endif /*SENSOR_LASER_2D_H_*/
