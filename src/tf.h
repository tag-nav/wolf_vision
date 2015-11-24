/*
 * TF.h
 *
 *  Created on: Nov 23, 2015
 *      Author: jsola
 */

#ifndef SRC_TF_H_
#define SRC_TF_H_

#include <eigen3/Eigen/Dense>

#include "wolf.h"

class TF
{
    private:
        unsigned int epoch_; ///< epoch counter to control updates automatically

    protected:
        Eigen::Vector3s t_; ///< Position vector
        Eigen::Vector4s q_vector_; ///< Orientation quaternion in vector form
        Eigen::Quaternions q_; ///< Orientation quaternion in quaternion form
        Eigen::Matrix3s R_; ///< Rotation matrix
        Eigen::Matrix4s H_; ///< Homogeneous matrix (aka. motion matrix)

    public:

        /**
         * Default constructor
         */
        TF() : epoch_(0),
               t_(Eigen::Vector3s::Zero()),
               q_vector_(0,0,0,0), // Impose a non-valid quaternion so that it will be updated for sure. This prevents a bad initialization due to different convention in the order of components.
               q_(Eigen::Quaternions::Identity()),
               R_(Eigen::Matrix3s::Identity()),
               H_(Eigen::Matrix4s::Identity())
    {
    }

        /**
         *
         */
        TF(double* _t_ptr, double* _q_ptr) :
            epoch_(0),
            t_(_t_ptr[0], _t_ptr[1], _t_ptr[2]),
            q_vector_(_q_ptr[0],_q_ptr[1],_q_ptr[2],_q_ptr[3]), //TODO: check components order!
            q_(_q_ptr), //TODO: check components order!
            R_(q_.matrix()),
            H_(Eigen::Matrix4s::Identity())
        {
            H_.block<3,3>(0,0) = R_;
            H_.block<3,1>(0,3) = t_;
        }

    public:
        void update(double* _t, double* _q_vector)
        {
            if (!equalT(_t))
            {
                t_(0) = _t[0];
                t_(1) = _t[1];
                t_(2) = _t[2];
                H_.block<3,1>(0,3) = t_;
            }
            if (!equalQ(_q_vector))
            {
                q_ = Eigen::Quaternion(_q_vector);
                R_ = q_.matrix();
                H_.block<3,3>(0,0) = R_;
            }
        }

        void update(double* _t, double* _q_vector, unsigned int _epoch)
        {
            if(_epoch != epoch_)
            {
                epoch_ = _epoch;
                t_(0) = _t[0];
                t_(1) = _t[1];
                t_(2) = _t[2];
                q_ = Eigen::Quaternion(_q_vector);
                R_ = q_.matrix();
                H_.block<3,3>(0,0) = R_;
                H_.block<3,1>(0,3) = t_;
            }
        }


    private:
        bool equalT(double * _t)
        {
            return ((_t[0] == t_(0)) && (_t[1] == t_(1)) && (_t[2] == t_(2)));
        }
        bool equalQ(double * _q)
        {
            return ((_q[0] == q_vector_(0)) && (_q[1] == q_vector_(1)) && (_q[2] == q_vector_(2)) && (_q[3] == q_vector_(3)));
        }
        void updateT(double * _t)
        {
            t_(0) = _t[0];
            t_(1) = _t[1];
            t_(2) = _t[2];
        }
        void updateO(double * _q_vector)
        {
            q_ = Eigen::Quaternion(_q_vector);
            R_ = q_.matrix();
            H_.block<3,3>(0,0) = R_;
            H_.block<3,1>(0,3) = t_;
        }

};

#endif /* SRC_TF_H_ */
