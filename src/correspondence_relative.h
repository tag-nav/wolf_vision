/**
 * \file correspondence_relative.h
 *
 *  Created on: 12/08/2014
 *     \author: jsola
 */

#ifndef CORRESPONDENCE_RELATIVE_H_
#define CORRESPONDENCE_RELATIVE_H_

#include "correspondence_base.h"

/**
 * \brief Relative constraint between two corresponding features.
 *
 * Relative constraint between two corresponding features.
 *
 * This type of constraint takes two frames, two sensors, and two feature measurements, and
 * computes the error of a relative constraint.
 *
 * Notice that there are some possibilities:
 * - Both frames are the same: it means we have two sensors on the robot from which we have seen the feature.
 * - Both sensors are the same: it means we have moved and are in another frame seeing the same feature.
 * - Frames and sensors are different: it means we observe from one sensor a feature that was seen in the past by another sensor.
 *
 * The constraint error is the disagreement between the expected measurement and the current measurement.
 * This disagreement is often defined by pure subtraction (and this class implements this default behavior),
 * but this is not mandatory and can be reimplemented in derived classes.
 *
 */
class CorrespondenceRelative : public CorrespondenceBase
{

    private:

        // Access "other" data, the corresponded feature.
        // Access all data of the corresponded feature using accessor functions.
        FeaturePtr feature_other_ptr_; ///< pointer to corresponded feature

    public:

        CorrespondenceRelative(const FeatureShPtr& _ft_ptr, const FeatureShPtr& _ft_other_ptr, unsigned int _dim_error,
                               unsigned int _dim_expectation);

        virtual ~CorrespondenceRelative();

        const StatePtr stateOtherPtr() const;
        const StatePose& stateOther() const;

        const CapturePtr captureOtherPtr() const;
        const Capture& captureOther() const;

        const SensorPtr sensorOtherPtr() const;
        const SensorBase& sensorOther() const;

        const FeaturePtr featureOtherPtr() const;
        const FeatureBase& featureOther() const;

        virtual void printSelf(unsigned int _ntabs = 0, std::ostream& _ost = std::cout) const;

};

//////////////////////////////////////
// IMPLEMENTATION
//////////////////////////////////////

CorrespondenceRelative::CorrespondenceRelative(const FeatureShPtr& _ft_ptr, const FeatureShPtr& _ft_other_ptr,
                                               unsigned int _dim_error, unsigned int _dim_expectation) :
        CorrespondenceBase(_ft_ptr, _dim_error, _dim_expectation), //
        feature_other_ptr_(_ft_other_ptr.get()) //
{
    //
}

CorrespondenceRelative::~CorrespondenceRelative()
{
    //
}

inline const StatePtr CorrespondenceRelative::stateOtherPtr() const
{
    return featureOtherPtr()->capturePtr()->framePtr()->statePtr();
}

inline const StatePose& CorrespondenceRelative::stateOther() const
{
    return featureOtherPtr()->capturePtr()->framePtr()->state();
}

inline const CapturePtr CorrespondenceRelative::captureOtherPtr() const
{
    return featureOtherPtr()->capturePtr();
}

inline const Capture& CorrespondenceRelative::captureOther() const
{
    return featureOtherPtr()->capture();
}

inline const SensorPtr CorrespondenceRelative::sensorOtherPtr() const
{
    return featureOtherPtr()->capturePtr()->sensorPtr();
}

inline const SensorBase& CorrespondenceRelative::sensorOther() const
{
    return featureOtherPtr()->capturePtr()->sensor();
}

inline const FeaturePtr CorrespondenceRelative::featureOtherPtr() const
{
    return feature_other_ptr_;
}

inline const FeatureBase& CorrespondenceRelative::featureOther() const
{
    return *feature_other_ptr_;
}

void CorrespondenceRelative::printSelf(unsigned int _ntabs, std::ostream& _ost) const
{
    CorrespondenceBase::printSelf(_ntabs, _ost);
    printNTabs(_ntabs);
    _ost << "\tCorr. Node   ~~> " << feature_other_ptr_->nodeId() << std::endl;
}

#endif /* CORRESPONDENCE_RELATIVE_H_ */
