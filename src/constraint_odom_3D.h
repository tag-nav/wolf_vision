/*
 * constraint_odom_3D.h
 *
 *  Created on: Oct 7, 2016
 *      Author: jsola
 */

#ifndef CONSTRAINT_ODOM_3D_H_
#define CONSTRAINT_ODOM_3D_H_

#include "constraint_sparse.h"
#include "rotations.h"

namespace wolf
{

class ConstraintOdom3D : public ConstraintSparse<6,3,4,3,4>
{
    public:
        ConstraintOdom3D(FeatureBasePtr _ftr_ptr, FrameBasePtr _frame_ptr, bool _apply_loss_function,
                         ConstraintStatus _status);
        virtual ~ConstraintOdom3D();

        template<typename T>
                bool operator ()(const T* const _p1,
                                 const T* const _q1,
                                 const T* const _p2,
                                 const T* const _q2,
                                 T* _residuals) const;

};

inline ConstraintOdom3D::ConstraintOdom3D(FeatureBasePtr _ftr_ptr, FrameBasePtr _frame_ptr, bool _apply_loss_function,
                                          ConstraintStatus _status) :
        ConstraintSparse<6, 3, 4, 3, 4>(CTR_ODOM_3D, _frame_ptr, _apply_loss_function, _status,
                                        _ftr_ptr->getFramePtr()->getPPtr(), // this frame P
                                        _ftr_ptr->getFramePtr()->getOPtr(), // this frame Q
                                        _frame_ptr->getPPtr(), // other frame P
                                        _frame_ptr->getOPtr()) // other frame Q
{
    std::cout << "Created Odom3D constraint" << std::endl;
    //
}

inline ConstraintOdom3D::~ConstraintOdom3D()
{
    //
}

template<typename T>
inline bool wolf::ConstraintOdom3D::operator ()(const T* const _p1, const T* const _q1, const T* const _p2,
                                                const T* const _q2, T* _residuals) const
{
    // MAPS
    Eigen::Map<const Eigen::Matrix<T,3,1> > p1(_p1);
    Eigen::Map<const Eigen::Quaternion<T> > q1(_q1);
    Eigen::Map<const Eigen::Matrix<T,3,1> > p2(_p2);
    Eigen::Map<const Eigen::Quaternion<T> > q2(_q2);
    Eigen::Map<Eigen::Matrix<T,10,1> > residuals(_residuals);

    // estimate motion increment, dp, dq;
    Eigen::Matrix<T,3,1> dp = q1.conjugate() * (p2 - p1);
    Eigen::Quaternion<T> dq = q1.conjugate() * q2;

    // measured motion increment, dp_m, dq_m
    Eigen::Map<Eigen::Matrix<T,3,1>> dp_m(getMeasurement().data());
    Eigen::Quaternion<T> dq_m = v2q(getMeasurement().tail<3>());

    // residual
    residuals.head<3>() = dp_m - dp;
    residuals.tail<3>() = q2v(dq.conjugate() * dq_m);
}

} /* namespace wolf */

#endif /* CONSTRAINT_ODOM_3D_H_ */
