
#ifndef SENSOR_ODOM_2D_H_
#define SENSOR_ODOM_2D_H_

//wolf includes
#include "sensor_base.h"

namespace wolf {

struct IntrinsicsOdom2D : public IntrinsicsBase
{
        Scalar k_disp_to_disp; ///< ratio of displacement variance to displacement, for odometry noise calculation
        Scalar k_rot_to_rot; ///< ratio of rotation variance to rotation, for odometry noise calculation
};

class SensorOdom2D : public SensorBase
{

    protected:
        Scalar k_disp_to_disp_; ///< ratio of displacement variance to displacement, for odometry noise calculation
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
		SensorOdom2D(StateBlock* _p_ptr, StateBlock* _o_ptr, const Scalar& _disp_noise_factor, const Scalar&  _rot_noise_factor);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         * 
         **/
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
        static SensorBase* create(const std::string& _unique_name, const Eigen::VectorXs& _extrinsics_pq, const IntrinsicsBase* _intrinsics);


};

} // namespace wolf

#endif // SENSOR_ODOM_2D_H_
