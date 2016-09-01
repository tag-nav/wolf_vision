#ifndef CONSTRAINT_IMU_THETA_H_
#define CONSTRAINT_IMU_THETA_H_

//Wolf includes
#include "constraint_sparse.h"
#include "feature_imu.h"
#include "frame_imu.h"

namespace wolf {

class ConstraintIMU : public ConstraintSparse<9, 3, 4, 3, 3, 3, 3, 4, 3>
{
    public:
        static const unsigned int N_BLOCKS = 8;

        ConstraintIMU(FeatureIMU* _ftr_ptr, FrameIMU* _frame_ptr, bool _apply_loss_function = false, ConstraintStatus _status = CTR_ACTIVE) :
                ConstraintSparse<9, 3, 4, 3, 3, 3, 3, 4, 3>(CTR_IMU, _frame_ptr, _apply_loss_function, _status,
                     _frame_ptr->getPPtr(), _frame_ptr->getOPtr(), _frame_ptr->getVPtr(), _frame_ptr->getBAPtr(), _frame_ptr->getBGPtr(),
                     _ftr_ptr->getFramePtr()->getPPtr(), _ftr_ptr->getFramePtr()->getOPtr(), _ftr_ptr->getFramePtr()->getVPtr())
        {
            setType("IMU");
        }

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~ConstraintIMU()
        {
            //
        }

        template<typename T>
        bool operator ()(const T* const _p1, const T* const _o1, const T* const _v1, const T* const _b_a1, const T* _b_g1,
                         const T* const _p2, const T* const _o2, const T* const _v2,
                         T* _residuals) const;

        /** \brief Returns the jacobians computation method
         **/
        virtual JacobianMethod getJacobianMethod() const
        {
            return JAC_AUTO;
        }

    public:
        static wolf::ConstraintBase* create(FeatureIMU* _feature_ptr, //
                                            NodeBase* _correspondant_ptr)
        {
            return new ConstraintIMU(_feature_ptr, (FrameIMU*)_correspondant_ptr);
        }

};

template<typename T>
inline bool ConstraintIMU::operator ()(const T* const _p1, const T* const _o1, const T* const _v1, const T* const _b_a1, const T* _b_g1,
                                       const T* const _p2, const T* const _o2, const T* const _v2,
                                       T* _residuals) const
{
    return true;
}

} // namespace wolf

#endif
