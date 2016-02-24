/*
 * LocalParametrizationHomogeneous.h
 *
 *  Created on: 24/02/2016
 *      Author: jsola
 */

#ifndef LOCALPARAMETRIZATIONHOMOGENEOUS_H_
#define LOCALPARAMETRIZATIONHOMOGENEOUS_H_

#include "local_parametrization_base.h"

class LocalParametrizationHomogeneous : public LocalParametrizationBase
{
    public:
        LocalParametrizationHomogeneous();
        virtual ~LocalParametrizationHomogeneous();

        virtual bool plus(const Eigen::Map<Eigen::VectorXs>& _h,
                          const Eigen::Map<Eigen::VectorXs>& _delta,
                          Eigen::Map<Eigen::VectorXs>& _h_plus_delta) const;
        virtual bool computeJacobian(const Eigen::Map<Eigen::VectorXs>& _h, Eigen::Map<Eigen::MatrixXs>& _jacobian) const;
};

#endif /* LOCALPARAMETRIZATIONHOMOGENEOUS_H_ */
