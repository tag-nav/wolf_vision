#ifndef LOCAL_PARAMETRIZATION_WRAPPER_H_
#define LOCAL_PARAMETRIZATION_WRAPPER_H_

// Fwd refs
namespace wolf{
class LocalParametrizationBase;
}

//Ceres includes
#include "base/wolf.h"
#include "ceres/ceres.h"

namespace wolf {

class LocalParametrizationWrapper : public ceres::LocalParameterization
{
    private:
        LocalParametrizationBasePtr local_parametrization_ptr_;

    public:

        LocalParametrizationWrapper(LocalParametrizationBasePtr _local_parametrization_ptr);

        virtual ~LocalParametrizationWrapper() = default;

        virtual bool Plus(const double* x_raw, const double* delta_raw, double* x_plus_delta_raw) const;

        virtual bool ComputeJacobian(const double* x, double* jacobian) const;

        virtual int GlobalSize() const;

        virtual int LocalSize() const;

        LocalParametrizationBasePtr getLocalParametrization() const;
};

using LocalParametrizationWrapperPtr = std::shared_ptr<LocalParametrizationWrapper>;

} // namespace wolf

#include "base/local_parametrization_base.h"

namespace wolf {

inline LocalParametrizationWrapper::LocalParametrizationWrapper(LocalParametrizationBasePtr _local_parametrization_ptr) :
        local_parametrization_ptr_(_local_parametrization_ptr)
{
}

inline int LocalParametrizationWrapper::GlobalSize() const
{
    return local_parametrization_ptr_->getGlobalSize();
}

inline int LocalParametrizationWrapper::LocalSize() const
{
    return local_parametrization_ptr_->getLocalSize();
}

inline LocalParametrizationBasePtr LocalParametrizationWrapper::getLocalParametrization() const
{
    return local_parametrization_ptr_;
}

} // namespace wolf

#endif
