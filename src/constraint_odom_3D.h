/*
 * constraint_odom_3D.h
 *
 *  Created on: Oct 7, 2016
 *      Author: jsola
 */

#ifndef CONSTRAINT_ODOM_3D_H_
#define CONSTRAINT_ODOM_3D_H_

#include "constraint_sparse.h"

namespace wolf
{

class ConstraintOdom3D : public ConstraintSparse<6,3,4,3,4>
{
    public:
        ConstraintOdom3D(FeatureBasePtr _ftr_ptr, FrameBasePtr _frame_ptr, bool _apply_loss_function,
                         ConstraintStatus _status);
        virtual ~ConstraintOdom3D();
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
    // TODO Auto-generated destructor stub
}

} /* namespace wolf */

#endif /* CONSTRAINT_ODOM_3D_H_ */
