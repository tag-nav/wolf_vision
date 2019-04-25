#ifndef SENSOR_ODOM_2D_H_
#define SENSOR_ODOM_2D_H_

//wolf includes
#include "base/sensor/sensor_base.h"
#include "base/utils/params_server.hpp"

namespace wolf {

WOLF_STRUCT_PTR_TYPEDEFS(IntrinsicsOdom2D);

struct IntrinsicsOdom2D : public IntrinsicsBase
{
        virtual ~IntrinsicsOdom2D() = default;

        Scalar k_disp_to_disp; ///< ratio of displacement variance to displacement, for odometry noise calculation
        Scalar k_rot_to_rot; ///< ratio of rotation variance to rotation, for odometry noise calculation
    IntrinsicsOdom2D()
    {
        //DEFINED FOR COMPATIBILITY PURPOSES. TO BE REMOVED IN THE FUTURE.
    }
    IntrinsicsOdom2D(std::string _unique_name, const paramsServer& _server):
        IntrinsicsBase(_unique_name, _server)
    {
        k_disp_to_disp = _server.getParam<Scalar>(_unique_name + "/k_disp_to_disp");
        k_rot_to_rot = _server.getParam<Scalar>(_unique_name + "/k_rot_to_rot");
    }
};

WOLF_PTR_TYPEDEFS(SensorOdom2D);

class SensorOdom2D : public SensorBase
{
    protected:
        Scalar k_disp_to_disp_; ///< ratio of displacement variance to displacement, for odometry noise calculation
        Scalar k_rot_to_rot_; ///< ratio of rotation variance to rotation, for odometry noise calculation

	public:
        SensorOdom2D(Eigen::VectorXs _extrinsics, const IntrinsicsOdom2D& _intrinsics);
        SensorOdom2D(Eigen::VectorXs _extrinsics, IntrinsicsOdom2DPtr _intrinsics);

        virtual ~SensorOdom2D();

        /** \brief Returns displacement noise factor
         *
         * Returns displacement noise factor
         *
         **/
        Scalar getDispVarToDispNoiseFactor() const;

        /** \brief Returns rotation noise factor
         *
         * Returns rotation noise factor
         *
         **/
        Scalar getRotVarToRotNoiseFactor() const;


	public:
        static SensorBasePtr create(const std::string& _unique_name, const Eigen::VectorXs& _extrinsics_pq, const IntrinsicsBasePtr _intrinsics);
        static SensorBasePtr createNew(const std::string& _unique_name, const paramsServer& _server);

};

} // namespace wolf

#endif // SENSOR_ODOM_2D_H_
