/**
 * \file constraint_range_bearing.h
 *
 *  Created on: Dec 1, 2017
 *      \author: jsola
 */

#ifndef HELLO_WOLF_CONSTRAINT_RANGE_BEARING_H_
#define HELLO_WOLF_CONSTRAINT_RANGE_BEARING_H_

#include "constraint_autodiff.h"

namespace wolf
{

WOLF_PTR_TYPEDEFS(ConstraintRangeBearing);

using namespace Eigen;

class ConstraintRangeBearing : public ConstraintAutodiff<ConstraintRangeBearing, 2, 2, 1, 2, 1, 2>
{
    public:
        ConstraintRangeBearing(const CaptureBasePtr& _capture_own,      // own capture's pointer
                               const LandmarkBasePtr& _landmark_other_ptr, // other landmark's pointer
                               const ProcessorBasePtr& _processor_ptr,  // processor having created this
                               bool _apply_loss_function,               // apply loss function to residual?
                               ConstraintStatus _status) :              // active constraint?
                    ConstraintAutodiff<ConstraintRangeBearing, 2, 2, 1, 2, 1, 2>( // sizes of: residual, rob pos, rob ori, sen pos, sen ori, lmk pos
                            CTR_BEARING_2D,                             // constraint type enum (see wolf.h)
                            nullptr,                                    // other frame's pointer
                            nullptr,                                    // other capture's pointer
                            nullptr,                                    // other feature's pointer
                            _landmark_other_ptr,                        // other landmark's pointer
                            _processor_ptr,                             // processor having created this
                            _apply_loss_function,                       // apply loss function to residual?
                            _status,                                    // active constraint?
                            _capture_own->getFramePtr()->getPPtr(),     // robot position
                            _capture_own->getFramePtr()->getOPtr(),     // robot orientation state block
                            _capture_own->getSensorPtr()->getPPtr(),    // sensor position state block
                            _capture_own->getSensorPtr()->getOPtr(),    // sensor orientation state block
                            _landmark_other_ptr->getPPtr())             // landmark position state block
        {
            setType("RANGE BEARING");                                   // constraint type text (for eventual ConstraintFactory and visualization)
        }

        virtual ~ConstraintRangeBearing()
        {
            //
        }

        template<typename T>
        bool operator ()(const T* const _p_w_r, // robot position
                         const T* const _o_w_r, // robot orientation
                         const T* const _p_r_s, // sensor position
                         const T* const _o_r_s, // sensor orientation
                         const T* const _lmk,   // landmark position
                         T* _res) const;        // residuals

};

} /* namespace wolf */


//////////////   IMPLEMENTATION   //////////////////////////////////

namespace wolf
{

template<typename T>
inline bool ConstraintRangeBearing::operator ()(const T* const _p_w_r, // robot position
                                                const T* const _o_w_r, // robot orientation
                                                const T* const _p_r_s, // sensor position
                                                const T* const _o_r_s, // sensor orientation
                                                const T* const _lmk,   // landmark position
                                                T* _res) const         // residuals
{
    // NOTE: The scalar type template 'T' can be of two types:
    //       - double     --> this allows direct computation of the residual
    //       - ceres::Jet --> this allows automatic computation of Jacobians of this function
    // it is the solver who calls operator() and decides on this type.
    //
    // The user needs to cast the obtained data to the proper type if necesssary:
    //       - Scalars: use (T)var or T(var) or (T)(var) to cast var into type T
    //       - Eigen types: use var.cast<T>() to cast var's inner scalars into type T
    // see the code for examples.

    // NOTE: This code here is very verbose. The steps for computing the residual are as follows:
    // 0. Arrange input data for practical usability
    // 1. Arrange TF transforms for practical usability
    // 2. Transform world-to-sensor
    // 3. Project to sensor and compute the expected measurement
    // 4. Get the actual measurement
    // 5. Compare actual and expected measurements, and compute the error
    // 6. Weight the error with the covariance, and compute the residual

    // 0. Map input pointers into meaningful Eigen elements
    Map<const Matrix<T, 2, 1>>      lmk(_lmk);      // point in world frame
    Map<Matrix<T, 2, 1>>            res(_res);      // residual

    // 1. produce transformation matrices to transform from sensor frame --> to robot frame --> to world frame
    Transform<T, 2, Affine> H_w_r = Translation<T,2>(_p_w_r[0], _p_w_r[1]) * Rotation2D<T>(*_o_w_r) ; // Robot  frame = robot-to-world transform
    Transform<T, 2, Affine> H_r_s = Translation<T,2>(_p_r_s[0], _p_r_s[1]) * Rotation2D<T>(*_o_r_s) ; // Sensor frame = sensor-to-robot transform

    // 2. Transform world-referenced landmark point to sensor-referenced point
    Transform<T, 2, Affine> H_w_s = H_w_r * H_r_s;  // world-to-sensor transform
    Matrix<T, 2, 1> lmk_s = H_w_s.inverse() * lmk;  // point in sensor frame

    // 3. Get the expected range-and-bearing of the point
    Matrix<T, 2, 1> exp_rb;
    exp_rb(0)      = sqrt(lmk_s.squaredNorm());        // range is norm. This code workaround is because Eigen::v.norm() is problematic with scalar type ceres::Jet
    exp_rb(1)      = atan2(lmk_s(1), lmk_s(0));        // bearing

    // 4. Get the measured range-and-bearing to the point
    Matrix<T, 2, 1> meas_rb       = getMeasurement().cast<T>(); // cast Eigen type vector to have scalar type 'T'

    // 5. Get the error by comparing the expected against the measurement
    Matrix<T, 2, 1> err_rb        = meas_rb - exp_rb;
    while (err_rb(1) < T(-M_PI))                        // bring angle between  -pi and pi
        err_rb(1) += T(2*M_PI);
    while (err_rb(1) > T(M_PI))
        err_rb(1) -= T(2*M_PI);

    // 6. Compute the residual by weighting the error according to the standard deviation of the bearing part
    // NOTE: the weight R is the upper square root of the information matrix Omega, which is in turn the inverse of the covariance Cov:
    //    R = Omega^(T/2) = Omega^(1/2)^T = Cov(-T/2)
    // where R is called the upper square root of Omega, and is such that
    //    R^T * R = Omega
    // in other words, R is the Cholesky decomposition of Omega.
    // NOTE: you get R directly from the Feature with getMeasurementSquareRootInformationUpper()
    res     = getMeasurementSquareRootInformationUpper().cast<T>() * err_rb;

    return true;
}

} // namespace wolf
#endif /* HELLO_WOLF_CONSTRAINT_RANGE_BEARING_H_ */
