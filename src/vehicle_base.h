/*
 * vehicle_base.h
 *
 *  Created on: Jun 10, 2014
 *      \author: jsola
 */

#ifndef VEHICLE_BASE_H_
#define VEHICLE_BASE_H_

#include "node_constrainer.h"


/*
 * forward declaration for parental links
 */
class Frame;
class NodeTerminus;
class SensorBase;

/**
 * \brief Base class for vehicles
 *
 * A vehicle is a moving platform equipped with sensors. It may be otherwise named 'robot' or any other
 * name suited for a given application.
 *
 * In the Wolf project, a vehicle is the topmost object in the data structure. It contains:
 *
 * - A \b collection \b of \b sensors, organized by sensor type. See SensorType for details.
 *
 *   Basically, the sensors have parameters
 *   (extrinsic and intrinsic) and a collection of methods for sensor data processing,
 *   organized in Feature Managers. See FeatureManagerBase for details.
 *
 * - A Trajectory of states, organized as a sliding window of keyframes. See Trajectory for details.
 *
 *   Basically, the Trajectory englobes all data that is changing with time. This includes the state of the
 *   vehicle (position, velocities, any dynamic state), the flow of data gathered by the sensors
 *   (images, GPS readings, IMU data), the result of processing this data (features, feature tracks,
 *   and their geometric measurements), and the correspondences of these features with other features
 *   in other frames and/or in data gathered by other sensors, usually of the same SensorType.
 *
 */
class VehicleBase : public NodeConstrainer<NodeTerminus, Frame>
{
    private:
//        Trajectory<StateType, OtherType> trajectory_; ///< Vehicle trajectory

    public:
        // protected: //TODO make protected once a derived Vehicle class exists

        VehicleBase(const NodeLocation _loc = TOP);

        /**
         * Destructor
         */
        virtual ~VehicleBase();

    public:
        void addFrame(const FrameShPtr& _frame_ptr);

        const FrameList& frameList() const;

};

/////////////////////////////////
// IMPLEMENTATION
/////////////////////////////////

inline VehicleBase::VehicleBase(const NodeLocation _loc) :
        NodeConstrainer<NodeTerminus, Frame>(_loc, "VEHICLE")
{
    //
}

inline VehicleBase::~VehicleBase()
{
}

inline void VehicleBase::addFrame(const FrameShPtr& _frame_ptr)
{
    addDownNode(_frame_ptr);
}

inline const FrameList& VehicleBase::frameList() const
{
    return downNodeList();
}

#endif /* VEHICLE_BASE_H_ */
