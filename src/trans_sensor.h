/**
 * \file trans_sensor.h
 *
 *  Created on: 31/08/2014
 *     \author: jsola
 */

#ifndef TRANS_SENSOR_H_
#define TRANS_SENSOR_H_

class Capture;

class TransSensor
{

    protected:
        CapturePtr capt_own_ptr_; ///< Pointer to the Capture owning this.
        CapturePtr capt_other_ptr_; ///< Pointer to the other Capture
        StatePose pose_other_to_own_; ///< The relative pose between captures, "other - to - this"

    public:
        TransSensor(const CapturePtr& _capt_own_ptr, const CapturePtr& _capt_other_ptr);
        virtual ~TransSensor();

        void precomputePoseRelative();
        const StatePose& poseRelative() const;

        // You should implement these two methods.
        // If you feel you do not need to implement precomputeConstants() then you probably do not need any TransSensor at all.
        /**
         * Compute reusable constant parameters forms from the set of parameters of both sensors.
         *
         * These parameter forms remain constant as long as the relative pose between captures does not change.
         */
        virtual void precomputeConstants() = 0;

        /**
         * Compute the expectation of a measurement in the own sensor of a feature that was observed in the other sensor.
         * @param _measurement_other the measurement in the other sensor.
         * @param _expectation the expected measurement in the own sensor. This is the return value.
         */
        virtual void computeExpectation(const Eigen::VectorXs & _measurement_other, Eigen::VectorXs & _expectation) const = 0;

};


///////////////////////////
// IMPLEMENTATION
///////////////////////////

#include "capture.h"
#include "wolf_tools.h"

inline TransSensor::TransSensor(const CapturePtr& _capt_own_ptr, const CapturePtr& _capt_other_ptr) :
        capt_own_ptr_(_capt_own_ptr), //
        capt_other_ptr_(_capt_other_ptr)
{
    //
//    std::cout << "Trans Sensor in capture " << capt_own_ptr_->nodeId() << " from other capture " << capt_other_ptr_->nodeId() << std::endl;
}

inline TransSensor::~TransSensor()
{
}

inline void TransSensor::precomputePoseRelative()
{
    Wolf::composeFrames(capt_other_ptr_->inverseGlobalPose(), capt_own_ptr_->globalPose(), pose_other_to_own_);
}

inline const StatePose& TransSensor::poseRelative() const
{
    return pose_other_to_own_;
}

#endif /* TRANS_SENSOR_H_ */
