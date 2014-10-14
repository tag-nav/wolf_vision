/**
 * \file correspondence_global.h
 *
 *  Created on: 12/08/2014
 *     \author: jsola
 */

#ifndef CORRESPONDENCE_GLOBAL_H_
#define CORRESPONDENCE_GLOBAL_H_

#include "correspondence_base.h"

class CorrespondenceGlobal : public CorrespondenceBase
{
        // TODO: add global data pointer, with getters.

    public:

        CorrespondenceGlobal(const FeatureShPtr& _ft_ptr, unsigned int _dim_error, unsigned int _dim_expectation);

        virtual ~CorrespondenceGlobal();

};

////////////////////////////////
// IMPLEMENTATION
////////////////////////////////


CorrespondenceGlobal::CorrespondenceGlobal(const FeatureShPtr& _ft_ptr, unsigned int _dim_error,
                                           unsigned int _dim_expectation) :
        CorrespondenceBase(_ft_ptr, _dim_error, _dim_expectation)
{
    //
}

CorrespondenceGlobal::~CorrespondenceGlobal()
{
    //
}

#endif /* CORRESPONDENCE_GLOBAL_H_ */
