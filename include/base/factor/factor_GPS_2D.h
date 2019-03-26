
#ifndef CONSTRAINT_GPS_2D_H_
#define CONSTRAINT_GPS_2D_H_

//Wolf includes
#include "base/wolf.h"
#include "base/factor/factor_autodiff.h"
#include "base/frame_base.h"

namespace wolf {
    
WOLF_PTR_TYPEDEFS(FactorGPS2D);

class FactorGPS2D : public FactorAutodiff<FactorGPS2D, 2, 2>
{
    public:

        FactorGPS2D(FeatureBasePtr _ftr_ptr, bool _apply_loss_function = false, FactorStatus _status = CTR_ACTIVE) :
            FactorAutodiff<FactorGPS2D, 2, 2>("GPS 2D", nullptr, nullptr, nullptr, nullptr, nullptr, _apply_loss_function, _status, _ftr_ptr->getFrame()->getP())
        {
            //
        }

        virtual ~FactorGPS2D() = default;

        template<typename T>
        bool operator ()(const T* const _x, T* _residuals) const;

};

template<typename T>
inline bool FactorGPS2D::operator ()(const T* const _x, T* _residuals) const
{
    _residuals[0] = (T(getMeasurement()(0)) - _x[0]) / T(sqrt(getMeasurementCovariance()(0, 0)));
    _residuals[1] = (T(getMeasurement()(1)) - _x[1]) / T(sqrt(getMeasurementCovariance()(1, 1)));
    return true;
}

} // namespace wolf

#endif
