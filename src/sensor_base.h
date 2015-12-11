#ifndef SENSOR_BASE_H_
#define SENSOR_BASE_H_

class HardwareBase;
class NodeTerminus;

//std includes
#include <iostream>

//Wolf includes
#include "wolf.h"
#include "node_linked.h"
#include "state_block.h"
#include "hardware_base.h"

class SensorBase : public NodeLinked<HardwareBase,NodeTerminus>
{
    protected:
//        SensorType type_;		// indicates sensor type. Enum defined at wolf.h
        StateBlock* p_ptr_;		// sensor position state block pointer
        StateBlock* o_ptr_; 	// sensor orientation state block pointer
        StateBlock* intrinsic_ptr_; // intrinsic parameters
        bool extrinsic_dynamic_;// extrinsic parameters vary with time? If so, they will be taken from the Capture nodes.

        Eigen::VectorXs noise_std_; // std of sensor noise
        Eigen::MatrixXs noise_cov_; // cov matrix of noise
        Eigen::VectorXs noise_factor_; // noise factor for simulation purposes -- multiplies the real sensor noise for the estimator

    public:        
        
        /** \brief Constructor with noise size
         *
         * Constructor with parameter vector
         * \param _tp Type of the sensor  (types defined at wolf.h)
         * \param _p_ptr StateBlock pointer to the sensor position
         * \param _o_ptr StateBlock pointer to the sensor orientation
         * \param _params Vector containing the sensor parameters
         *
         **/
        SensorBase(const SensorType & _tp, StateBlock* _p_ptr, StateBlock* _o_ptr, StateBlock* _intr_ptr, const unsigned int _noise_size, const bool _extr_dyn = false);

        /** \brief Constructor with noise std vector
         *
         * Constructor with parameter vector
         * \param _tp Type of the sensor  (types defined at wolf.h)
         * \param _p_ptr StateBlock pointer to the sensor position
         * \param _o_ptr StateBlock pointer to the sensor orientation
         * \param _params Vector containing the sensor parameters
         *
         **/
        SensorBase(const SensorType & _tp, StateBlock* _p_ptr, StateBlock* _o_ptr, StateBlock* _intr_ptr, const Eigen::VectorXs & _noise_std, const bool _extr_dyn = false);

//        /** \brief Constructor with parameter size
//         *
//         * Constructor with parameter vector
//         * \param _tp Type of the sensor  (types defined at wolf.h)
//         * \param _p_ptr StateBlock pointer to the sensor position
//         * \param _o_ptr StateBlock pointer to the sensor orientation
//         * \param _params_size size of the vector containing the sensor parameters
//         *
//         **/
//        SensorBase(const SensorType & _tp, StateBlock* _p_ptr, StateBlock* _o_ptr, unsigned int _params_size, const bool _extr_dyn = false);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~SensorBase();

//        const SensorType getSensorType() const;

        StateBlock* getPPtr() const;

        StateBlock* getOPtr() const;

        StateBlock* getIntrinsicPtr() const;

        Eigen::Matrix2s getRotationMatrix2D();

        Eigen::Matrix3s getRotationMatrix3D();

        void fix();

        void unfix();

        /**
         * Check if sensor is dynamic
         */
        bool isExtrinsicDynamic();

        void setNoise(const Eigen::VectorXs & _noise_std);

        Eigen::VectorXs getNoiseStd();

        Eigen::MatrixXs getNoiseCov();

};
#endif

