/*
 * sensor_data_base.h
 *
 *  Created on: July 11, 2014
 *      \author: acorominas
 */

#ifndef SENSOR_DATA_BASE_H_
#define SENSOR_DATA_BASE_H_

//std
#include <memory>

//wolf
#include "wolf.h"
#include "frame.h"
//#include "sensor_base.h"
#include "raw_base.h"
#include "feature_base.h"

//namespaces
using namespace std;
using namespace Eigen;

// forward declaration for tree nodes
class Frame;
class FeatureBase;

/** \brief Class for sensor data 
 *
 * Placeholder for sensor data to be stored. It is placed below Frames at the Wolf tree. 
 * It may finish the tree (DownNode=NodeTerminus), or have below a map of Features.
 * It has two main members:
 * \param sensor_ptr_ weak pointer to a sensor setup (extrinsic and intrinsic params)
 * \param raw_ptr_ shared pointer to raw data.
 * Template parameters indicates the type of each of the above pointers. 
 * 
 */
template<class RawType> //, class SensorType>
class SensorDataBase : public NodeConstrainer<Frame, FeatureType, FeatureBase>
{
    protected:
        //weak_ptr<SensorType> sensor_; ///< Pointer to sensor
        shared_ptr<RawType> raw_ptr_; ///< Pointer to raw data
        
    public:
        /** \brief Constructor 
         * 
         * Constructor 
         * \param _loc Placement of the node within the Wolf tree. See enum NodeLocation defined at file wolf.h.
         * \param _dim dimension of the error vector.
         * 
         */
        SensorDataBase(const NodeLocation _loc, const unsigned int _dim, shared_ptr<RawType> _raw_ptr) :
            NodeConstrainer<Frame, FeatureType, FeatureBase>(_loc, _dim), 
            raw_ptr_(_raw_ptr)
        {
            //
        };

        /** \brief Destructor
         * 
         * Destructor
         * 
         */
        ~SensorDataBase()
        {
            //
        };
        
        /** \brief Returns a reference to time stamp
         * 
         * Returns a reference to time stamp
         * 
         */
        TimeStamp & timeStamp()
        {
            return raw_ptr_->->timeStamp();
        };
};
#endif /* SENSOR_DATA_BASE_H_ */
