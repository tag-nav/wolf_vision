/*
 * Gaussian.h
 *
 *  Created on: May 15, 2014
 *      \author: jsola
 */

#ifndef Gaussian_H_
#define Gaussian_H_

//eigen
#include <eigen3/Eigen/Dense> 

//wolf
#include "density_base.h"

using namespace std;
using namespace Eigen;

/**
 * \brief Gaussian probability density
 * 
 * Gaussian probability density, with a mean vector and a covariances matrix.
 * 
 */
class Gaussian :public DensityBase
{
    protected:
        MatrixXs covariance_;
        
    public:

        /**
         * Size constructor
         * @param _size dimension of the random vector
         */
        Gaussian(const unsigned int _size);

        /**
         * Destructor
         */
        virtual ~Gaussian();
        
        MatrixXs covariance(); ///< the covariances matrix
};
#endif /* Gaussian_H_ */
