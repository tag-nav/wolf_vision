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
        ConstraintRangeBearing(const LandmarkBasePtr& _landmark_other_ptr,
                               const ProcessorBasePtr& _processor_ptr,
                               bool _apply_loss_function, ConstraintStatus _status) :
                                   ConstraintAutodiff<ConstraintRangeBearing, 2, 2, 1, 2, 1, 2>(CTR_BEARING_2D, nullptr, nullptr, nullptr,
                                                                                                _landmark_other_ptr, _processor_ptr,
                                                                                                _apply_loss_function, _status,
                                                                                                getCapturePtr()->getFramePtr()->getPPtr(),
                                                                                                getCapturePtr()->getFramePtr()->getOPtr(),
                                                                                                getCapturePtr()->getSensorPtr()->getPPtr(),
                                                                                                getCapturePtr()->getSensorPtr()->getOPtr(),
                                                                                                _landmark_other_ptr->getPPtr())
        {
            //
        }

        virtual ~ConstraintRangeBearing()
        {
            //
        }

        template<typename T>
        bool operator ()(const T* const _p_w_r,
                         const T* const _o_w_r,
                         const T* const _p_r_s,
                         const T* const _o_r_s,
                         const T* const _lmk,
                         T* _res) const;

};

} /* namespace wolf */


//////////////   IMPLEMENTATION   //////////////////////////////////

namespace wolf
{

template<typename T>
inline bool ConstraintRangeBearing::operator ()(const T* const _p_w_r,
                                                const T* const _o_w_r,
                                                const T* const _p_r_s,
                                                const T* const _o_r_s,
                                                const T* const _lmk,
                                                T* _res) const
{
    // NOTE: This code here is very verbose

    // Map input pointers into meaningful Eigen elements
    Map<const Matrix<T, 2, 1>>      lmk(_lmk);      // point in world frame
    Map<Matrix<T, 2, 1>>            res(_res);      // residual

    // 1. produce transformation matrices to transform from sensor frame to robot frame to world frame
    Transform<T, 2, Affine> H_w_r = Translation<T,2>(_p_w_r[0], _p_w_r[1]) * Rotation2D<T>(*_o_w_r) ; // Robot  frame = robot-to-world transform
    Transform<T, 2, Affine> H_r_s = Translation<T,2>(_p_r_s[0], _p_r_s[1]) * Rotation2D<T>(*_o_r_s) ; // Sensor frame = sensor-to-robot transform

    // 2. Transform world point to sensor-referenced point
    Transform<T, 2, Affine> H_w_s = H_w_r * H_r_s;
    Matrix<T, 2, 1> lmk_s = H_w_s.inverse() * lmk;  // point in sensor frame

    // 3. Get the expected range-and-bearing of the point
    Matrix<T, 2, 1> exp;
    exp(0)      = sqrt(lmk_s.squaredNorm());        // range is norm. This code workaround is because Eigen::v.norm() is problematic with scalar type ceres::Jet
    exp(1)      = atan2(lmk_s(1), lmk_s(0));        // bearing

    // 4. Get the measured range-and-bearing to the point
    Matrix<T, 2, 1> range_bearing       = getMeasurement().cast<T>();

    // 5. Get the error by comparing the expected against the measurement
    Matrix<T, 2, 1> er(range_bearing - exp);
    if (er(1) < T(-M_PI))
        er(1) += T(2*M_PI);
    else if  (er(1) > T(-M_PI))
        er(1) -= T(2*M_PI);

    // 6. Compute the residual by weighting the error according to the standard deviation of the bearing part
    res     = getMeasurementSquareRootInformationUpper().cast<T>() * er;

    return true;
}

} // namespace wolf
#endif /* HELLO_WOLF_CONSTRAINT_RANGE_BEARING_H_ */
