/*
 * DensityBase.h
 *
 *  Created on: May 15, 2014
 *      \author: jsola
 */

#ifndef DensityBase_H_
#define DensityBase_H_

// wolf
#include "wolf.h"

using namespace std;
using namespace Eigen;

/**
 * \brief Base class for probability densities
 * 
 * Base class for probability densities. It has a mean value, which acts as
 * some kind of nominal value, but no specification of uncertainty.
 * 
 */
class DensityBase
{
    protected:
        VectorXs mean_; ///< the mean value
        
    public:

        /**
         * Constructor from size
         * @param _size dimension of the random variable
         */
        DensityBase(const unsigned int _size);

        /**
         * Destructor
         */
        virtual ~DensityBase();
        
        /**
         * The mean value of the probability density or distribution
         * @return the mean value
         */
        VectorXs& mean();
        
};
#endif /* DensityBase_H_ */
