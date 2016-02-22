#ifndef CONSTRAINT_POINT_2D_H
#define CONSTRAINT_POINT_2D_H

#include "constraint_sparse.h"

class ConstraintPoint2D : public ConstraintSparse<3, 2, 1, 2, 1>
{
    public:
        static const unsigned int N_BLOCKS = 4;

        ConstraintPoint2D(FeatureBase* _ftr_ptr, FrameBase* _frame_ptr, ConstraintStatus _status = CTR_ACTIVE) :
                ConstraintSparse<3, 2, 1, 2, 1>(_ftr_ptr, CTR_ODOM_2D, _frame_ptr, _status, _frame_ptr->getPPtr(), _frame_ptr->getOPtr(), _ftr_ptr->getFramePtr()->getPPtr(), _ftr_ptr->getFramePtr()->getOPtr())
        {                                                 //esta variable se cambia en wolf.h
            //
        }

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~ConstraintPoint2D()
        {
            //
        }
}




#endif // CONSTRAINT_POINT_2D_H
