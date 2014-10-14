/*
 * feature_manager_base.h
 *
 *  Created on: Jun 10, 2014
 *      \author: jsola
 */

#ifndef FEATURE_MANAGER_BASE_H_
#define FEATURE_MANAGER_BASE_H_

#include "feature_base.h"

/**
 * \brief Base class for feature managers
 *
 * A feature manager is the responsible for processing the raw data to produce a particular kind of feature,
 * and to evaluate metrics these features such as expected locations and innovations. It contains mainly processes:
 *
 * - detect() Detect features in the raw data
 * - match() Associate features to a previously given set of features.
 * - expectation() compute the expected measurement of a particular feature
 * - innovation() compare the real measurement to the expected measurement
 * - cost() evaluate a certain norm of the innovation to be interpreted as an estimation error.
 */
class FeatureManagerBase
{
    private:
        FeatureType feat_type_;         ///< Feature type
    public:
        FeatureManagerBase();
        virtual ~FeatureManagerBase();
};

#endif /* FEATURE_MANAGER_BASE_H_ */
