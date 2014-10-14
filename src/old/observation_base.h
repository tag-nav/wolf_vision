/*
 * time_stamp.h
 *
 *  Created on: May 26, 2014
 *      \author: acorominas@iri.upc.edu
 */

#ifndef observation_base_H
#define observation_base_H

//std
#include <string>
#include <memory>

//eigen
#include <eigen3/Eigen/Dense> 

//wolf
#include "time_stamp.h"
#include "state_base.h"
#include "density_base.h"

using namespace std;
using namespace Eigen;

/**
 * \brief Base class for observations
 */

template<class DensityType>
class ObservationBase
{
    protected:
        TimeStamp ts_; ///< time stamp
        DensityType measurement_;
        DensityType expectation_;
        DensityType innovation_;

    public:
        /**
         * Constructor from size
         */
        ObservationBase(const unsigned int _size);

        /**
         * Destructor
         */
        virtual ~ObservationBase();

        /**
         * Compute expected measurement
         */
        virtual VectorXs& computeExpectation(StateRootBase & _s) = 0;
//        VectorXs & computeExpectation(StateBase & _s){ return expectation_.mean(); };// = 0;

        /**
         * Compute innovation
         */
        virtual VectorXs& computeInnovation(StateRootBase & _s) = 0;
//        VectorXs & computeInnovation(StateBase & _s){ return innovation_.mean(); };// = 0;

        /**
         * Compute cost of the innovation
         */
        virtual scalar_t computeCost(StateRootBase & _s) = 0;
//        scalar_t computeCost(StateBase & _s){return 0;};// = 0;
};

typedef std::shared_ptr<ObservationBase<DensityBase> > ObservationBasePtr;

template<class DensityType>
inline ObservationBase<DensityType>::ObservationBase(const unsigned int _size) :
        measurement_(_size), expectation_(_size), innovation_(_size)
{
}

template<class DensityType>
inline ObservationBase<DensityType>::~ObservationBase()
{
}

#endif
