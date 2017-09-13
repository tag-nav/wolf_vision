
#ifndef CONSTRAINT_GPS_2D_H_
#define CONSTRAINT_GPS_2D_H_

//Wolf includes
#include "wolf.h"
#include "constraint_autodiff.h"
#include "frame_base.h"

namespace wolf {
    
WOLF_PTR_TYPEDEFS(ConstraintGPS2D);

class ConstraintGPS2D : public ConstraintAutodiff<ConstraintGPS2D, 2, 2>
{
    public:

        ConstraintGPS2D(FeatureBasePtr _ftr_ptr, bool _apply_loss_function = false, ConstraintStatus _status = CTR_ACTIVE) :
            ConstraintAutodiff<ConstraintGPS2D, 2, 2>(CTR_GPS_FIX_2D, nullptr, nullptr, nullptr, nullptr, _apply_loss_function, _status, _ftr_ptr->getFramePtr()->getPPtr())
        {
            setType("GPS FIX 2D");
        }

        virtual ~ConstraintGPS2D() = default;

        template<typename T>
        bool operator ()(const T* const _x, T* _residuals) const;

        /** \brief Returns the jacobians computation method
         *
         * Returns the jacobians computation method
         *
         **/
        virtual JacobianMethod getJacobianMethod() const override
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

} // namespace wolf

#endif
