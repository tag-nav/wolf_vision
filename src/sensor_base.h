/*
 * \file sensor_base.h
 *
 *  Created on: May 15, 2014
 *      \author: jsola
 */

/**
 * \file sensor_base.h
 */

#ifndef SENSOR_BASE_H_
#define SENSOR_BASE_H_

// forward declarations
class NodeTerminus;
class VehicleBase;

//wolf includes
#include "state_pose.h"
#include "node_linked.h"

/** \brief Base class for all kind of sensors.
 * 
 * This class is the base for all sensor classes.
 * 
 */
class SensorBase : public NodeLinked<VehicleBase, NodeTerminus>
{
    private:
        StatePose local_pose_; ///< Local sensor pose
        StateBase params_; ///< intrinsic parameters

    protected:
        /** Constructor from sensor pose
         *
         * \param _local_pose the sensor pose within the mobile platform
         * \param _loc = MID the location in the tree. TOP, MID, BOTTOM. Defaults to MID.
         *
         */
        SensorBase(const StatePose& _local_pose, const NodeLocation _loc = MID);
       
        /** Constructor from sensor pose and intrinsic params
         *
         * \param _local_pose the sensor pose within the mobile platform
         * \param _intrinsic the intrinsic parameters
         * \param _loc = MID the location in the tree. TOP, MID, BOTTOM. Defaults to MID.
         *
         */
        SensorBase(const StatePose& _local_pose, const Eigen::VectorXs _intrinsic, const NodeLocation _loc = MID);

        /**
         * Destructor
         */
        virtual ~SensorBase();

    public:

        /**
         * Get a reference to the intrinsic vector
         * \return the vector of intrinsic parameters
         */
        const Eigen::Map<Eigen::VectorXs>& intrinsic() const;

        /**
         * Get a reference to the sensor pose
         * \return the sensor pose. This is an instance of SensorPose, with at least position p() and orientation q().
         */
        const StatePose& pose() const;

};

////////////////////////////
//  IMPLEMENTATION
////////////////////////////

SensorBase::SensorBase(const StatePose& _local_pose, const NodeLocation _loc) :
        NodeLinked(_loc, "SENSOR"), //
        local_pose_(_local_pose), //
        params_(0)
{
    //
}

SensorBase::SensorBase(const StatePose& _local_pose, const Eigen::VectorXs _intrinsic, const NodeLocation _loc) :
        NodeLinked(_loc, "SENSOR"), //
        local_pose_(_local_pose), //
        params_(_intrinsic)
{
    //
}

inline SensorBase::~SensorBase()
{
    //
}

inline const Eigen::Map<Eigen::VectorXs>& SensorBase::intrinsic() const
{
    return params_.x();
}

inline const StatePose& SensorBase::pose() const
{
    return local_pose_;
}

#endif /* SENSOR_BASE_H_ */
