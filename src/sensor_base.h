#ifndef SENSOR_BASE_H_
#define SENSOR_BASE_H_

//std includes
#include <iostream>

//Wolf includes
#include "wolf.h"
#include "node_base.h"
#include "state_base.h"

class SensorBase : public NodeBase
{
    protected:
        SensorType type_;		//indicates sensor type. Enum defined at wolf.h
        StateBase* p_ptr_;		// sensor position state unit pointer
        StateBase* o_ptr_; 	    // sensor orientation state unit pointer
        Eigen::VectorXs params_;//sensor intrinsic params: offsets, scale factors, sizes, ...

    public:        
        /** \brief Constructor without parameters
         *
         * Constructor without parameters. 
         * Allows allocation of containers of StateBase, such as std::vector<SensorBase>
         *
         **/
        //SensorBase();
        
        /** \brief Constructor with parameter vector
         *
         * Constructor with parameter vector
         * \param _tp Type of the sensor  (types defined at wolf.h)
         * \param _p_ptr StateBase pointer to the sensor position
         * \param _o_ptr StateBase pointer to the sensor orientation
         * \param _params Vector containing the sensor parameters
         *
         **/
        SensorBase(const SensorType & _tp, StateBase* _p_ptr, StateBase* _o_ptr, const Eigen::VectorXs & _params);

        /** \brief Constructor with parameter size
         *
         * Constructor with parameter vector
         * \param _tp Type of the sensor  (types defined at wolf.h)
         * \param _p_ptr StateBase pointer to the sensor position
         * \param _o_ptr StateBase pointer to the sensor orientation
         * \param _params_size size of the vector containing the sensor parameters
         *
         **/
        SensorBase(const SensorType & _tp, StateBase* _p_ptr, StateBase* _o_ptr, unsigned int _params_size);

        ~SensorBase();

        const SensorType getSensorType() const;

        StateBase* getPPtr() const;

        StateBase* getOPtr() const;

        Eigen::Matrix2s getRotationMatrix2D();

        Eigen::Matrix3s getRotationMatrix3D();

};
#endif

