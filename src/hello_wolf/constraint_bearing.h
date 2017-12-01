/*
 * ConstraintBearing.h
 *
 *  Created on: Nov 30, 2017
 *      Author: jsola
 */

#ifndef HELLO_WOLF_CONSTRAINT_BEARING_H_
#define HELLO_WOLF_CONSTRAINT_BEARING_H_

#include "constraint_autodiff.h"
#include "rotations.h"

namespace wolf
{

using namespace Eigen;

class ConstraintBearing : public ConstraintAutodiff<ConstraintBearing, 1, 2, 1, 2>
{
    public:
        ConstraintBearing(const LandmarkBasePtr& _landmark_other_ptr,
                          const ProcessorBasePtr& _processor_ptr,
                          bool _apply_loss_function,
                          ConstraintStatus _status);
        virtual ~ConstraintBearing();

        template<typename T>
        bool operator ()(const T* const _p1,
                         const T* const _o1,
                         const T* const _p2,
                         T* _res) const;

};

} /* namespace wolf */

template<typename T>
inline bool wolf::ConstraintBearing::ConstraintBearing::operator ()(const T* const _p1, const T* const _o1,
                                                                    const T* const _p2, T* _res) const
{

    // 1. produce a transformation matrix to transform from robot frame to world frame
    Transform<T, 2, Affine> H_w_r = Translation<T,2>(_p1[0], _p1[1]) * Rotation2D<T>(*_o1) ; // Robot frame = robot-to-world transform
    // Map input pointers into meaningful Eigen elements
    Map<const Matrix<T, 2, 1>>      point_w(_p2);
    Map<const Matrix<T, 1, 1>>      res(_res);

    // 2. Transform world point to robot-referenced point
    Matrix<T, 2, 1> point_r = H_w_r.inverse() * point_w;

    // 3. Get the expected bearing of the point
    T bearing   = atan2(point_r(1), point_r(0));

    // 4. Get the measured range-and-bearing to the point, and its covariance
    Matrix<T, 2, 1> range_bearing       = getMeasurement().cast<T>();
    Matrix<T, 2, 2> range_bearing_cov   = getMeasurementCovariance().cast<T>();

    // 5. Get the bearing error by comparing against the bearing measurement
    T er   = range_bearing(1) - bearing;
    if (er < T(-M_PI))
        er += T(2*M_PI);
    else if  (er > T(-M_PI))
        er -= T(2*M_PI);

    // 6. Compute the residual by scaling according to the standard deviation of the bearing part
    T sigma = sqrt(range_bearing_cov(1,1));
    *_res   = er / sigma;

    return true;
}

#endif /* HELLO_WOLF_CONSTRAINT_BEARING_H_ */
