/**
 * \file sensor_odom_3D.h
 *
 *  Created on: Oct 7, 2016
 *      \author: jsola
 */

#ifndef SRC_SENSOR_ODOM_3D_H_
#define SRC_SENSOR_ODOM_3D_H_

//wolf includes
#include "sensor_base.h"

namespace wolf {

struct IntrinsicsOdom3D : public IntrinsicsBase
{
        typedef std::shared_ptr<IntrinsicsOdom3D> Ptr;

        Scalar k_disp_to_disp; ///< ratio of displacement variance to displacement, for odometry noise calculation
        Scalar k_disp_to_rot; ///< ratio of displacement variance to rotation, for odometry noise calculation
        Scalar k_rot_to_rot; ///< ratio of rotation variance to rotation, for odometry noise calculation
        Scalar min_disp_var;
        Scalar min_rot_var;
        Scalar max_buffer_length;
        Scalar dist_traveled;
        Scalar angle_turned;

        IntrinsicsOdom3D() :
            k_disp_to_disp(0),
            k_disp_to_rot(0),
            k_rot_to_rot(0),
            min_disp_var(0),
            min_rot_var(0),
            max_buffer_length(0),
            dist_traveled(0),
            angle_turned(0)
        {}
};

class SensorOdom3D : public SensorBase
{
    public:
        typedef std::shared_ptr<SensorOdom3D> Ptr;

    protected:
        Scalar k_disp_to_disp_; ///< ratio of displacement variance to displacement, for odometry noise calculation
        Scalar k_disp_to_rot_; ///< ratio of displacement variance to rotation, for odometry noise calculation
        Scalar k_rot_to_rot_; ///< ratio of rotation variance to rotation, for odometry noise calculation
        Scalar min_disp_var_;
        Scalar min_rot_var_;

        Scalar max_buffer_length_;
        Scalar dist_traveled_;
        Scalar angle_turned_;

    public:
        /** \brief Constructor with arguments
         *
         * Constructor with arguments
         * \param _p_ptr StateBlock pointer to the sensor position wrt vehicle base
         * \param _o_ptr StateBlock pointer to the sensor orientation wrt vehicle base
         * \param _disp_noise_factor displacement noise factor
         * \param _rot_noise_factor rotation noise factor
         *
         **/
        SensorOdom3D(StateBlockPtr _p_ptr, StateQuaternionPtr _q_ptr, IntrinsicsOdom3D::Ptr params);

        virtual ~SensorOdom3D();

        Scalar getDispVarToDispNoiseFactor() const;
        Scalar getDispVarToRotNoiseFactor() const;
        Scalar getRotVarToRotNoiseFactor() const;
        Scalar getMinDispVar() const;
        Scalar getMinRotVar() const;

        Scalar getMaxBufferLength() const;
        Scalar getDistTraveled() const;
        Scalar getAngleTurned() const;


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

inline Scalar SensorOdom3D::getMaxBufferLength() const
{
    return max_buffer_length_;
}

inline Scalar SensorOdom3D::getDistTraveled() const
{
    return dist_traveled_;
}

inline Scalar SensorOdom3D::getAngleTurned() const
{
    return angle_turned_;
}

} /* namespace wolf */

#endif /* SRC_SENSOR_ODOM_3D_H_ */
