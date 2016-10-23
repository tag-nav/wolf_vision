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
        Scalar k_disp_to_disp; ///< ratio of displacement variance to displacement, for odometry noise calculation
        Scalar k_disp_to_rot; ///< ratio of displacement variance to rotation, for odometry noise calculation
        Scalar k_rot_to_rot; ///< ratio of rotation variance to rotation, for odometry noise calculation
};

class SensorOdom3D : public SensorBase
{

    protected:
        Scalar k_disp_to_disp_; ///< ratio of displacement variance to displacement, for odometry noise calculation
        Scalar k_disp_to_rot_; ///< ratio of displacement variance to rotation, for odometry noise calculation
        Scalar k_rot_to_rot_; ///< ratio of rotation variance to rotation, for odometry noise calculation

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
        SensorOdom3D(StateBlockPtr _p_ptr, StateQuaternionPtr _q_ptr, const Scalar& _k_disp_to_disp, const Scalar& _k_disp_to_rot, const Scalar&  _k_rot_to_rot);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~SensorOdom3D();

        /** \brief Returns displacement noise factor
         *
         * Returns displacement noise factor
         *
         **/
        Scalar getDispVarToDispNoiseFactor() const;

        /** \brief Returns displacement noise factor
         *
         * Returns displacement noise factor
         *
         **/
        Scalar getDispVarToRotNoiseFactor() const;

        /** \brief Returns rotation noise factor
         *
         * Returns rotation noise factor
         *
         **/
        Scalar getRotVarToRotNoiseFactor() const;


    public:
        static SensorBasePtr create(const std::string& _unique_name, const Eigen::VectorXs& _extrinsics_pq, const IntrinsicsBasePtr _intrinsics);


};

} /* namespace wolf */

#endif /* SRC_SENSOR_ODOM_3D_H_ */
