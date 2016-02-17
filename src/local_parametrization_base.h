/*
 * \file local_parametrization_base.h
 *
 *  Created on: Feb 17, 2016
 *      author: jsola
 */

#ifndef SRC_LOCAL_PARAMETRIZATION_BASE_H_
#define SRC_LOCAL_PARAMETRIZATION_BASE_H_


class LocalParametrizationBase{
    private:
        unsigned int size_global_;
        unsigned int size_local_;
        LocalParametrizationBase();
        virtual ~LocalParametrizationBase();
        bool plus(Eigen::VectorXs& _x_in, Eigen::VectorXs& _dx, Eigen::VectorXs& _x_out);
        bool jacobian();
};


#endif /* SRC_LOCAL_PARAMETRIZATION_BASE_H_ */
