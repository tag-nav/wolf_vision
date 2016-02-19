/*
 * \file local_parametrization_base.h
 *
 *  Created on: Feb 17, 2016
 *      \author: jsola
 */

#ifndef LOCAL_PARAMETRIZATION_BASE_H_
#define LOCAL_PARAMETRIZATION_BASE_H_

#include "wolf.h"


class LocalParametrizationBase{
    private:
        unsigned int global_size_;
        unsigned int local_size_;
    public:
        LocalParametrizationBase(unsigned int _global_size, unsigned int _local_size);
        virtual ~LocalParametrizationBase();

        virtual bool plus(const Eigen::Map<Eigen::VectorXs>& _x,
                          const Eigen::Map<Eigen::VectorXs>& _delta,
                          Eigen::Map<Eigen::VectorXs>& _x_plus_delta) const = 0;
        virtual bool computeJacobian(const Eigen::Map<Eigen::VectorXs>& _x, Eigen::Map<Eigen::MatrixXs>& _jacobian) const = 0;

        unsigned int getLocalSize();
        unsigned int getGlobalSize();
};
#endif /* LOCAL_PARAMETRIZATION_BASE_H_ */
