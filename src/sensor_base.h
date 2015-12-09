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
        SensorType type_;		// indicates sensor type. Enum defined at wolf.h
        StateBlock* p_ptr_;		// sensor position state block pointer
        StateBlock* o_ptr_; 	// sensor orientation state block pointer
        Eigen::VectorXs params_;// sensor intrinsic params: offsets, scale factors, sizes, ...
        bool dynamic_extrinsic_;// extrinsic parameters vary with time. They will be taken from the Capture nodes.

    public:        
        
        /** \brief Constructor with parameter vector
         *
         * Constructor with parameter vector
         * \param _tp Type of the sensor  (types defined at wolf.h)
         * \param _p_ptr StateBlock pointer to the sensor position
         * \param _o_ptr StateBlock pointer to the sensor orientation
         * \param _params Vector containing the sensor parameters
         *
         **/
        SensorBase(const SensorType & _tp, StateBlock* _p_ptr, StateBlock* _o_ptr, const Eigen::VectorXs & _params);

        /** \brief Constructor with parameter size
         *
         * Constructor with parameter vector
         * \param _tp Type of the sensor  (types defined at wolf.h)
         * \param _p_ptr StateBlock pointer to the sensor position
         * \param _o_ptr StateBlock pointer to the sensor orientation
         * \param _params_size size of the vector containing the sensor parameters
         *
         **/
        SensorBase(const SensorType & _tp, StateBlock* _p_ptr, StateBlock* _o_ptr, unsigned int _params_size);

        virtual ~SensorBase();

        const SensorType getSensorType() const;

        StateBlock* getPPtr() const;

        StateBlock* getOPtr() const;

        Eigen::Matrix2s getRotationMatrix2D();

        Eigen::Matrix3s getRotationMatrix3D();

        void fix();

        void unfix();

        /**
         * Make this sensor dynamic so that extrinsics vary with time.
         */
        void setDynamic(bool _dyn = true);

        /**
         * Make this sensor static so that extrinsics do not vary with time
         */
        void unsetDynamic();

        /**
         * Check if sensor is dynamic
         */
        bool isDynamic();

};
#endif

