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

class ConstraintRangeBearing : public ConstraintAutodiff<ConstraintRangeBearing, 2, 2, 1, 2>
{
    public:
        ConstraintRangeBearing(const LandmarkBasePtr& _landmark_other_ptr,
                               const ProcessorBasePtr& _processor_ptr,
                               bool _apply_loss_function, ConstraintStatus _status) :
                     ConstraintAutodiff<ConstraintBearing, 2, 2, 1, 2>(CTR_BEARING_2D, nullptr, nullptr, nullptr,
                                                                       _landmark_other_ptr, _processor_ptr,
                                                                       _apply_loss_function, _status,
                                                                       getCapturePtr()->getFramePtr()->getPPtr(),
                                                                       getCapturePtr()->getFramePtr()->getOPtr(),
                                                                       _landmark_other_ptr->getPPtr())
        {
            //
        }

        virtual ~ConstraintRangeBearing()
        {
            // TODO Auto-generated destructor stub
        }

        template<typename T>
        bool operator ()(const T* const _p1,
                         const T* const _o1,
                         const T* const _p2,
                         T* _res) const;

};

} /* namespace wolf */

namespace wolf
{

template<typename T>
inline bool ConstraintRangeBearing::operator ()(const T* const _p1, const T* const _o1,
                                                                    const T* const _p2, T* _res) const
{

    // 1. produce a transformation matrix to transform from robot frame to world frame
    Transform<T, 2, Affine> H_w_r = Translation<T,2>(_p1[0], _p1[1]) * Rotation2D<T>(*_o1) ; // Robot frame = robot-to-world transform
    // Map input pointers into meaningful Eigen elements
    Map<const Matrix<T, 2, 1>>      point_w(_p2);
    Map<const Matrix<T, 1, 1>>      res(_res);

    // 2. Transform world point to robot-referenced point
    Matrix<T, 2, 1> point_r = H_w_r.inverse() * point_w;

    // 3. Get the expected range and bearing of the point
    Matrix<T, 2, 1> exp;
    exp(0)      = sqrt(point_r.squaredNorm());
    exp(1)      = atan2(point_r(1), point_r(0));

    // 4. Get the measured range-and-bearing to the point, and its covariance
    Matrix<T, 2, 1> range_bearing       = getMeasurement().cast<T>();
    Matrix<T, 2, 2> range_bearing_cov   = getMeasurementCovariance().cast<T>();

    // 5. Get the bearing error by comparing against the bearing measurement
    Matrix<T, 2, 1> er(range_bearing - exp);
    if (er(1) < T(-M_PI))
        er(1) += T(2*M_PI);
    else if  (er(1) > T(-M_PI))
        er(1) -= T(2*M_PI);

    // 6. Compute the residual by scaling according to the standard deviation of the bearing part
    res     = getMeasurementSquareRootInformationTransposed().cast<T>() * er;

    return true;
}

} // namespace wolf
#endif /* HELLO_WOLF_CONSTRAINT_RANGE_BEARING_H_ */
