/**
 * \file sensor_odom_3D.h
 *
 *  Created on: Oct 7, 2016
 *      \author: jsola
 */

#ifndef SRC_SENSOR_ODOM_3D_H_
#define SRC_SENSOR_ODOM_3D_H_

//wolf includes
#include "base/sensor/sensor_base.h"

namespace wolf {

WOLF_STRUCT_PTR_TYPEDEFS(IntrinsicsOdom3D);

struct IntrinsicsOdom3D : public IntrinsicsBase
{
        Scalar k_disp_to_disp; ///< ratio of displacement variance to displacement, for odometry noise calculation
        Scalar k_disp_to_rot; ///< ratio of displacement variance to rotation, for odometry noise calculation
        Scalar k_rot_to_rot; ///< ratio of rotation variance to rotation, for odometry noise calculation
        Scalar min_disp_var;
        Scalar min_rot_var;

        virtual ~IntrinsicsOdom3D() = default;
};

WOLF_PTR_TYPEDEFS(SensorOdom3D);

class SensorOdom3D : public SensorBase
{
    protected:
        Scalar k_disp_to_disp_; ///< ratio of displacement variance to displacement, for odometry noise calculation
        Scalar k_disp_to_rot_; ///< ratio of displacement variance to rotation, for odometry noise calculation
        Scalar k_rot_to_rot_; ///< ratio of rotation variance to rotation, for odometry noise calculation
        Scalar min_disp_var_;
        Scalar min_rot_var_;

    public:
        SensorOdom3D(const Eigen::VectorXs& _extrinsics_pq, const IntrinsicsOdom3D& params);
        SensorOdom3D(const Eigen::VectorXs& _extrinsics_pq, IntrinsicsOdom3DPtr params);

        virtual ~SensorOdom3D();

        Scalar getDispVarToDispNoiseFactor() const;
        Scalar getDispVarToRotNoiseFactor() const;
        Scalar getRotVarToRotNoiseFactor() const;
        Scalar getMinDispVar() const;
        Scalar getMinRotVar() const;

    public:
        static SensorBasePtr create(const std::string& _unique_name, const Eigen::VectorXs& _extrinsics_pq, const IntrinsicsBasePtr _intrinsics);

};

inline Scalar SensorOdom3D::getDispVarToDispNoiseFactor() const
{
    return k_disp_to_disp_;
}

inline Scalar SensorOdom3D::getDispVarToRotNoiseFactor() const
{
    return k_disp_to_rot_;
}

inline Scalar SensorOdom3D::getRotVarToRotNoiseFactor() const
{
    return k_rot_to_rot_;
}

inline Scalar SensorOdom3D::getMinDispVar() const
{
    return min_disp_var_;
}

inline Scalar SensorOdom3D::getMinRotVar() const
{
    return min_rot_var_;
}

} /* namespace wolf */

#endif /* SRC_SENSOR_ODOM_3D_H_ */
