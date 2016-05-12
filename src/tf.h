/*
 * TF.h
 *
 *  Created on: Nov 23, 2015
 *      Author: jsola
 */

#ifndef SRC_TF_H_
#define SRC_TF_H_

// Wolf includes
#include "wolf.h"

// Std includes
#include <eigen3/Eigen/Dense>

namespace wolf
{

/**
 * Class for 3D frame transforms.
 *
 * This is a simple class to manage frame updates so that rotation and homogeneous matrices
 * are computed only once for the lifetime of a specific TF parametrization.
 *
 * Unit quaternions are used for 3D orientation input. They require the real part at the end.
 */
class TF
{
    private:
        unsigned int epoch_; ///< epoch counter to control updates automatically

    protected:
        Eigen::Vector3s p_; ///< Position vector
        Eigen::Vector4s q_vector_; ///< Orientation quaternion in vector form
        Eigen::Quaternions q_; ///< Orientation quaternion in quaternion form
        Eigen::Matrix3s R_; ///< Rotation matrix
        Eigen::Matrix4s H_; ///< Homogeneous matrix (aka. motion matrix)

    public:

        /**
         * Default constructor to the origin, P=(0,0,0), Q=(0,0,0,1), R=identity, H=identity
         */
        TF() :
                epoch_(0), p_(Eigen::Vector3s::Zero()), //
                q_vector_(0, 0, 0, 1), // Impose an identity quaternion with real part at the end. This is Eigen compatible.
                q_(Eigen::Quaternions::Identity()), //
                R_(Eigen::Matrix3s::Identity()), //
                H_(Eigen::Matrix4s::Identity())
        {
        }

        /**
         * Initializer constuctor
         * \param _p_ptr pointer to position vector
         * \param _q_vec_ptr pointer to quaternion vector (real part at the end)
         */
        TF(double* _p_ptr, double* _q_vec_ptr) :
                epoch_(0),
                // p_(_p_ptr[0], _p_ptr[1], _p_ptr[2]),
                // q_vector_(_q_vec_ptr[0],_q_vec_ptr[1],_q_vec_ptr[2],_q_vec_ptr[3]), //
                p_(_p_ptr), //
                q_vector_(_q_vec_ptr), //
                q_(_q_vec_ptr), //
                R_(q_.matrix()), //
                H_(Eigen::Matrix4s::Identity())
        {
            H_.block<3, 3>(0, 0) = R_;
            H_.block<3, 1>(0, 3) = p_;
        }

    public:
        void checkAndUpdate(double* _p_ptr, double* _q_vec_ptr)
        {
            if (!equalP(_p_ptr))
                updateP(_p_ptr);
            if (!equalQ(_q_vec_ptr))
                updateO(_q_vec_ptr);
        }

        void checkAndUpdate(double* _p_ptr, double* _q_vec_ptr, unsigned int _epoch)
        {
            if (_epoch != epoch_)
            {
                epoch_ = _epoch;
                updateP(_p_ptr);
                updateO(_q_vec_ptr);
            }
        }

    private:
        /** Check if P has not changed
         *
         */
        bool equalP(double * _p_ptr)
        {
            return ((_p_ptr[0] == p_(0)) && (_p_ptr[1] == p_(1)) && (_p_ptr[2] == p_(2)));
        }

        /** Check if Q has not changed
         *
         */
        bool equalQ(double * _q_vec_ptr)
        {
            return ((_q_vec_ptr[0] == q_vector_(0)) && (_q_vec_ptr[1] == q_vector_(1))
                    && (_q_vec_ptr[2] == q_vector_(2)) && (_q_vec_ptr[3] == q_vector_(3)));
        }

        /** Update translation vector
         *
         */
        void updateP(double * _p_ptr)
        {
            p_(0) = _p_ptr[0];
            p_(1) = _p_ptr[1];
            p_(2) = _p_ptr[2];
            H_.block<3, 1>(0, 3) = p_;
        }

        /** Update orientation parts
         *
         */
        void updateO(double * _q_vec_ptr)
        {
            q_vector_(0) = _q_vec_ptr[0];
            q_vector_(1) = _q_vec_ptr[1];
            q_vector_(2) = _q_vec_ptr[2];
            q_vector_(3) = _q_vec_ptr[3];
            q_ = Eigen::Quaternion(_q_vec_ptr);
            R_ = q_.matrix();
            H_.block<3, 3>(0, 0) = R_;
        }

};

} // namespace wolf

#endif /* SRC_TF_H_ */
