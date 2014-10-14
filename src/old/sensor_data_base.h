/*
 * sensor_data_base.h
 *
 *  Created on: Jun 10, 2014
 *      \author: jsola
 */

#ifndef SENSOR_DATA_BASE_H_
#define SENSOR_DATA_BASE_H_

#include "parent_of.h"
#include "child_of.h"
#include "link_list.h"

#include "sensor_base.h"
#include "raw_base.h"
#include "feature_base.h" // only for FeatureType definition

//namespaces
using namespace std;

// forward declaration for parental links
class Frame;
class FeatureBase;

//classes
/**
 * \brief Base class for sensor data
 *
 * Data captured by sensors, and the result of the processing of this data, are collected in this class.
 * the idea is to have an object Sensor for each physical sensor, several objects FeatureManager to process
 * the captured data, and one SensorData object for each capture of the sensor. In the case of trajectories,
 * a number of captures are active inside the sliding window of keyframes.
 *
 * SensorData has:
 * - Raw data,
 * - A pointer to the Sensor that captured the data.
 * - A list of features (points, lines, GPS pseudo-ranges, or whatsoever).
 * - A link_list, a list of pointers to older or current nodes with correspondence in terms of data association
 * - A link_list, a list of pointers to forward nodes that correspond to this
 *
 * See RawBase for a rationale on raw data.
 *
 * See SensorBase for a rationale on sensor devices.
 *
 * See FeatureBase for a rationale on features.
 *
 */
class SensorDataBase : public ParentOf<FeatureType,FeatureBase>, public ChildOf<Frame>
{
    public:
        /** \brief Pointer to the sensor device
         *
         * Pointer to the sensor device used to get this raw data
         * 
         **/
        weak_ptr<SensorBase> sensor_; 
        
        /** \brief Pointer to the raw data
         * 
         * Pointer to the raw data used to extract features
         * 
         **/
        shared_ptr<RawBase> raw_;
        
        
        /** \brief List of constraint links
         * 
         * LinkList of pointers to past and current SensorDataBase nodes that this node stablishes a 
         * correspondence with. These links are relevant to the optimizer. 
         * 
         **/
        LinkList<SensorDataBase> constraint_node_list_;
        
        /** \brief List of forward node links
         * 
         * LinkList of pointers to future SensorDataBase nodes which they stablish a correspondence with this. 
         * Useful at destructor time to properly clear constraint node lists os these nodes 
         * These links are not relevant to the optimizer.
         * 
         **/        
        LinkList<SensorDataBase> forward_node_list_;
        
    public:
        /** \brief Constructor from sensor pointer
         * 
         * Constructor from sensor pointer
         * \param _sensor_ptr pointer to the sensor device
         * \param _raw_ptr pointer to the raw data (sensor reading)
         * 
         */
        SensorDataBase(const weak_ptr<SensorBase> & _sensor_ptr, const shared_ptr<RawBase> & _raw_ptr);
        
        /** \brief Destructor
         *
         * Destructor
         * 
         */
        virtual ~SensorDataBase();

        /** \brief Generic feature detector
         * 
         * Generic feature detector
         * 
         */
        void detect();
        
        /** \brief Checks if matchable
         * 
         * Checks wether raw data in this class can be matched with raw data owned by the argument 
         * 
         */        
        bool isMatchable(const shared_ptr<SensorDataBase> & _sensor_ptr) const;
        
        /** \brief Stablish matches
         * 
         * Stablish matches between raw data of this object and raw data of argument
         * 
         */        
        void match(const shared_ptr<SensorDataBase>& _sensor_ptr);
};
#endif /* SENSOR_DATA_BASE_H_ */
