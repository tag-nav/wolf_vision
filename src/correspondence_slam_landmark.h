/**
 * \file correspondence_slam_landmark.h
 *
 *  Created on: 12/08/2014
 *     \author: jsola
 */

#ifndef CORRESPONDENCE_SLAM_LANDMARK_H_
#define CORRESPONDENCE_SLAM_LANDMARK_H_

#include "correspondence_base.h"

class CorrespondenceSLAMLandmark : public CorrespondenceBase
{
        // TODO: add SLAM Landmark data pointer, with their getters.

    public:

        CorrespondenceSLAMLandmark(const FeatureShPtr& _ft_ptr, unsigned int _dim_error, unsigned int _dim_expectation);

        virtual ~CorrespondenceSLAMLandmark();

};

////////////////////////////////
// IMPLEMENTATION
////////////////////////////////



CorrespondenceSLAMLandmark::CorrespondenceSLAMLandmark(const FeatureShPtr& _ft_ptr, unsigned int _dim_error,
                                                       unsigned int _dim_expectation) :
        CorrespondenceBase(_ft_ptr, _dim_error, _dim_expectation)
{
    //
}

CorrespondenceSLAMLandmark::~CorrespondenceSLAMLandmark()
{
    //
}

#endif /* CORRESPONDENCE_SLAM_LANDMARK_H_ */
