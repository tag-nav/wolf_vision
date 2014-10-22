/**
 * \file capture.h
 *
 *  Created on: 12/08/2014
 *     \author: jsola
 */

#ifndef CAPTURE_H_
#define CAPTURE_H_


// Forward declarations for node templates
class Frame;
class FeatureBase;

// Forward declarations for member pointers
class SensorBase;
class RawBase;
class TransSensor;

//wolf includes
#include "node_constrainer.h"
#include "state_pose.h"

//std includes
#include <map>
#include <utility>
#include <memory>
#include <iostream>


/** \brief Base class for raw data captures from sensors
 *
 * Raw data captured by sensors, and links to the sensors parameters, are collected in this class.
 *
 * The idea is to have an object Sensor for each physical sensor, several objects FeatureManager to process
 * the captured data, and one Capture object for each actual raw data captured by the sensor. In the case of trajectories,
 * a number of captures are active inside a sliding window of key-frames. In this context, usually each key-frame
 * has at least one sensor capture, but it is allowed to have more than one (from different sensors
 * fired at the same time-stamp of the Frame).
 *
 * One object Capture has:
 * - Raw data,
 * - A pointer to the Sensor that captured the data
 * - A pointer to the Frame owning it (one level up in the Wolf tree).
 * - A std::list of features (points, lines, GPS pseudo-ranges, or whatsoever), the result of interpreting the raw data
 *   (one level down in the Wolf tree).
 *
 * See RawBase for a rationale on raw data.
 *
 * See SensorBase for a rationale on sensor devices.
 *
 * See Frame for a rationale on frames and key-frames.
 *
 * See FeatureBase for a rationale on features.
 *
 */
class Capture : public NodeConstrainer<Frame, FeatureBase>
{

    protected:
        RawShPtr raw_sh_ptr_; ///< Pointer to raw data. This Capture is owner of raw data. 
        SensorPtr sensor_ptr_; ///< Pointer to sensor
        StatePose global_pose_; ///< Global sensor pose, the composition of the frame pose and the sensor pose
        StatePose inverse_global_pose_; ///< inverse global sensor pose

    private:
        TransSensorMap trans_sensor_map_;

    protected:
        /** \brief Constructor
         * 
         * \param _frm_ptr a shared pointer to the Frame up node
         * \param _sen_ptr a shared pointer to the sensor that captured the data
         * \param _loc the location in the Wolf tree (TOP, MID or BOTTOM, it defaults to MID)
         * 
         */
        Capture(const FrameShPtr& _frm_ptr, const SensorShPtr& _sen_ptr, const NodeLocation _loc = MID);

        /** \brief Destructor
         * 
         * Destructor
         *
         **/        
        virtual ~Capture();

    public:
        /** \brief Set link to Frame
         * 
         * Set link to Frame
         *
         **/        
        void linkToFrame(const FrameShPtr& _frm_ptr);

        /** \brief Gets up_node_ptr_
         * 
         * Gets up_node_ptr_, which is a pointer to the Frame owning of this Capture
         *
         **/                
        const FramePtr framePtr() const;
        
        /** \brief Gets a reference of Frame
         * 
         * Gets a reference of the Frame owning of this Capture
         *
         **/                
        const Frame& frame() const;

        /** \brief Adds a Feature to the down node list
         * 
         * Adds a Feature to the down node list
         *
         **/                        
        void addFeature(const FeatureShPtr& _f_ptr);

        /** \brief Gets a const reference to feature list
         * 
         * Gets a const reference to feature list
         *
         **/                        
        const FeatureList& featureList() const;

        /** \brief Gets a const ref to Raw shared pointer
         * 
         * Gets a const ref to Raw shared pointer
         *
         **/                        
        const RawShPtr& rawShPtr() const;
        
        /** \brief Gets a const pointer to Raw
         * 
         * Gets a const pointer to Raw
         *
         **/                                
        const RawPtr rawPtr() const;
        
        /** \brief Sets Raw pointer from a shared pointer
         * 
         * Sets Raw pointer from a shared pointer
         *
         **/                                
        void setRaw(const RawShPtr & _raw_sh_ptr);

        /** \brief Gets a pointer to SensorBase
         * 
         * Gets a pointer to SensorBase
         *
         **/                        
        SensorPtr sensorPtr() ;
        SensorBase& sensor() ;
        const SensorPtr sensorPtr() const;
        const SensorBase& sensor() const;

        void precomputeConstants();

        void precomputeGlobalPose();
        const StatePose & globalPose() const;
        const StatePose & inverseGlobalPose() const;

        bool existsTransSensor(unsigned int _capture_other_id) const;
        std::pair<TransSensorIter,bool> addTransSensor(unsigned int _capture_other_id, const TransSensorShPtr& _trans_senspr_sh_ptr);

        /** \brief Get the TransSensor pointer corresponding to an ID key.
         * 
         * @param _capture_other_id the TransSensor identifier
         * @return the TransSensor pointer. Return NULL if TransSensor ID is not found.
         *
         * Get the TransSensor pointer corresponding to an ID key.
         */
        const TransSensorPtr transSensorPtr(unsigned int _capture_other_id) const;

        /** @brief Erases elements according to the provided key.
         *  
         *  @param  _capture_other_id  Key of element to be erased.
         *  @return  The number of elements erased.
         *
         *  This function erases all the elements located by the given key from a %map.
         *  Note that this function only erases the element, and that if
         *  the element is itself a pointer, the pointed-to memory is not touched
         *  in any way.  Managing the pointer is the user's responsibility.
         */
        unsigned int removeTransSensor(unsigned int _capture_other_id);
        
        /** \brief Generic processing from raw to features
         * 
         * Generic processing from raw to features. 
         * 
         **/
        virtual void processCapture();

        virtual void printSelf(unsigned int _ntabs = 0, std::ostream& _ost = std::cout) const;

};

////////////////////////////////
// IMPLEMENTATION
////////////////////////////////

#include "trans_sensor.h"
#include "sensor_base.h"
#include "wolf_tools.h"

Capture::Capture(const FrameShPtr& _frm_ptr, const SensorShPtr& _sen_ptr, const NodeLocation _loc) :
        NodeConstrainer(_loc, "CAPTURE", _frm_ptr), //
        sensor_ptr_(_sen_ptr.get())
{
    //
}

Capture::~Capture()
{
    //
}

inline void Capture::linkToFrame(const FrameShPtr& _frm_ptr)
{
    linkToUpNode(_frm_ptr);
}

inline const FramePtr Capture::framePtr() const
{
    return upNodePtr();
}

inline const Frame& Capture::frame() const
{
    return *framePtr();
}

inline void Capture::addFeature(const FeatureShPtr& _f_ptr)
{
    addDownNode(_f_ptr);
}

const inline FeatureList& Capture::featureList() const
{
    return downNodeList();
}

inline const RawShPtr& Capture::rawShPtr() const
{
    return raw_sh_ptr_;
}

inline const RawPtr Capture::rawPtr() const
{
    return raw_sh_ptr_.get();
}

inline void Capture::setRaw(const RawShPtr & _raw_sh_ptr)
{
    raw_sh_ptr_ = _raw_sh_ptr;
}

inline SensorPtr Capture::sensorPtr()
{
    return sensor_ptr_;
}

inline SensorBase& Capture::sensor()
{
    return *sensor_ptr_;
}

inline const SensorPtr Capture::sensorPtr() const
{
    return sensor_ptr_;
}

inline const SensorBase& Capture::sensor() const
{
    return *sensor_ptr_;
}

inline void Capture::precomputeConstants()
{
    precomputeGlobalPose();
    for (auto const & ptr : trans_sensor_map_)
        ptr.second->precomputeConstants();
}

inline void Capture::precomputeGlobalPose()
{
        Wolf::composeFrames(framePtr()->state(), sensor_ptr_->pose(), global_pose_ );
        global_pose_.inverse(inverse_global_pose_);
}

inline const StatePose& Capture::globalPose() const
{
    return global_pose_;
}

inline const StatePose& Capture::inverseGlobalPose() const
{
    return inverse_global_pose_;
}

inline bool Capture::existsTransSensor(unsigned int _capture_other_id) const
{
    return trans_sensor_map_.count(_capture_other_id) != 0;
}

inline std::pair<TransSensorIter,bool> Capture::addTransSensor(unsigned int _capture_other_id, const TransSensorShPtr& _trans_senspr_sh_ptr)
{
    auto ret =  trans_sensor_map_.insert(TransSensorMap::value_type(_capture_other_id, _trans_senspr_sh_ptr));
    return ret;
}

inline const TransSensorPtr Capture::transSensorPtr(unsigned int _capture_other_id) const
{
    // NOTE: We allow a search by capture ID because the total number of trans-sensors in a Capture is really small
    // and a smarter, quicker, random access is not justified.
    auto iter = trans_sensor_map_.find(_capture_other_id);
    if (iter->first == _capture_other_id)
        //        if (iter != trans_sensor_map_.end()) // alternate test, they work similarly
        return iter->second.get(); // this is a regular pointer to the TransSensor.
    else
    {
        return NULL; // Return a NULL pointer if the TransSensor ID is not found.
    }
}

inline unsigned int Capture::removeTransSensor(unsigned int _capture_other_id)
{
    return trans_sensor_map_.erase(_capture_other_id);
}

inline void Capture::processCapture()
{
    //nothing to do in base class. Overload it to do things.
}

void Capture::printSelf(unsigned int _ntabs, std::ostream& _ost) const
{
    NodeConstrainer::printSelf(_ntabs, _ost);
    printNTabs(_ntabs);
    _ost << "\tSensor pose : ( " << sensor_ptr_->pose().x().transpose() << " )" << std::endl;
    printNTabs(_ntabs);
    _ost << "\tSensor intrinsic : ( " << sensor_ptr_->intrinsic().transpose() << " )" << std::endl;
    printNTabs(_ntabs);
    _ost << "\tTransSensors ==> [ ";
    for (auto const & ptr : trans_sensor_map_)
    {
        _ost << ptr.first << " "; // print the capture ID key of the TransSensor
    }
    _ost << "]" << std::endl;
}

#endif /* CAPTURE_H_ */
