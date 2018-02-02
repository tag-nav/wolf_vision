/*
 * \file local_parametrization_base.h
 *
 *  Created on: Feb 17, 2016
 *      \author: jsola
 */

#ifndef LOCAL_PARAMETRIZATION_BASE_H_
#define LOCAL_PARAMETRIZATION_BASE_H_

#include "wolf.h"


namespace wolf {

class LocalParametrizationBase{
    protected:
        unsigned int global_size_;
        unsigned int local_size_;
    public:
        LocalParametrizationBase(unsigned int _global_size, unsigned int _local_size);
        virtual ~LocalParametrizationBase();

        virtual bool plus(Eigen::Map<const Eigen::VectorXs>& _x,
                          Eigen::Map<const Eigen::VectorXs>& _delta,
                          Eigen::Map<Eigen::VectorXs>& _x_plus_delta) const = 0;
        virtual bool computeJacobian(Eigen::Map<const Eigen::VectorXs>& _x, Eigen::Map<Eigen::MatrixXs>& _jacobian) const = 0;
        virtual bool minus(Eigen::Map<const Eigen::VectorXs>& _x1,
                           Eigen::Map<const Eigen::VectorXs>& _x2,
                           Eigen::Map<Eigen::VectorXs>& _x2_minus_x1) = 0;

        unsigned int getLocalSize() const;
        unsigned int getGlobalSize() const;
};

} // namespace wolf

#endif /* LOCAL_PARAMETRIZATION_BASE_H_ */
