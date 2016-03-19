
#ifndef CONSTRAINT_GPS_2D_H_
#define CONSTRAINT_GPS_2D_H_

//Wolf includes
#include "wolf.h"
#include "constraint_sparse.h"

class ConstraintGPS2D : public ConstraintSparse<2, 2>
{
    public:
        static const unsigned int N_BLOCKS = 1;

        ConstraintGPS2D(FeatureBase* _ftr_ptr, FrameBase* _frame_ptr, ConstraintStatus _status = CTR_ACTIVE) :
                ConstraintSparse<2, 2>(_ftr_ptr, CTR_GPS_FIX_2D, _status, _frame_ptr->getPPtr())
        {
            //
        }

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~ConstraintGPS2D()
        {
            //
        }

        template<typename T>
        bool operator ()(const T* const _x, T* _residuals) const;

        /** \brief Returns the jacobians computation method
         *
         * Returns the jacobians computation method
         *
         **/
        virtual JacobianMethod getJacobianMethod() const
        {
            return JAC_AUTO;
        }
};

template<typename T>
inline bool ConstraintGPS2D::operator ()(const T* const _x, T* _residuals) const
{
    _residuals[0] = (T(getMeasurement()(0)) - _x[0]) / T(sqrt(getMeasurementCovariance()(0, 0)));
    _residuals[1] = (T(getMeasurement()(1)) - _x[1]) / T(sqrt(getMeasurementCovariance()(1, 1)));
    return true;
}

#endif
