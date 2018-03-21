// sensor_laser_2D.h

#ifndef SENSOR_LASER_2D_H_
#define SENSOR_LASER_2D_H_

//wolf
#include "sensor_base.h"

//laser_scan_utils
#include "laser_scan_utils/laser_scan.h"

// std

namespace wolf {

WOLF_STRUCT_PTR_TYPEDEFS(IntrinsicsLaser2D);

struct IntrinsicsLaser2D : public IntrinsicsBase
{
        virtual ~IntrinsicsLaser2D() = default;

        laserscanutils::LaserScanParams scan_params;
};

WOLF_PTR_TYPEDEFS(SensorLaser2D);

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
        SensorLaser2D(StateBlockPtr _p_ptr, StateBlockPtr _o_ptr);

        /** \brief Constructor with extrinsics and scan parameters
         *
         * \param _p_ptr StateBlock pointer to the sensor position
         * \param _o_ptr StateBlock pointer to the sensor orientation
         *
         **/
        SensorLaser2D(StateBlockPtr _p_ptr, StateBlockPtr _o_ptr, const double& _angle_min, const double& _angle_max, const double& _angle_step, const double& _scan_time, const double& _range_min, const double& _range_max, const double& _range_std_dev, const double& _angle_std_dev);

        /** \brief Constructor with extrinsics and scan parameters
         *
         * \param _p_ptr StateBlock pointer to the sensor position
         * \param _o_ptr StateBlock pointer to the sensor orientation
         * \param _params Scan parameters
         *
         **/
        SensorLaser2D(StateBlockPtr _p_ptr, StateBlockPtr _o_ptr, const laserscanutils::LaserScanParams& _params);

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
        static SensorBasePtr create(const std::string& _unique_name, const Eigen::VectorXs& _extrinsics_po, const IntrinsicsBasePtr _intrinsics);
        static IntrinsicsBasePtr createParams(const std::string& _filename_dot_yaml);

};

} // namespace wolf

#endif /*SENSOR_LASER_2D_H_*/
