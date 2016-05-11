
#ifndef CONSTRAINT_FIX_H_
#define CONSTRAINT_FIX_H_

//Wolf includes
#include "wolf.h"
#include "constraint_sparse.h"


namespace wolf {

class ConstraintFix: public ConstraintSparse<3,2,1>
{
    public:
        static const unsigned int N_BLOCKS = 2;

        ConstraintFix(FeatureBase* _ftr_ptr, bool _apply_loss_function = false, ConstraintStatus _status = CTR_ACTIVE) :
                ConstraintSparse<3, 2, 1>(_ftr_ptr, CTR_FIX, _apply_loss_function, _status, _ftr_ptr->getFramePtr()->getPPtr(),
                                          _ftr_ptr->getFramePtr()->getOPtr())
        {
            setType("FIX");
            //std::cout << "creating ConstraintFix " << std::endl;
        }

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~ConstraintFix()
        {
            //
        }

        template<typename T>
        bool operator ()(const T* const _p, const T* const _o, T* _residuals) const;

        /** \brief Returns the jacobians computation method
         *
         * Returns the jacobians computation method
         *
         **/
        virtual JacobianMethod getJacobianMethod() const
        {
            return JAC_AUTO;
        }

    public:
        static wolf::ConstraintBase* create(FeatureBase* _feature_ptr, //
                NodeBase* _correspondant_ptr = nullptr)
        {
            return new ConstraintFix(_feature_ptr);
        }

};

template<typename T>
inline bool ConstraintFix::operator ()(const T* const _p, const T* const _o, T* _residuals) const
{
    //std::cout << "computing constraint odom ..." << std::endl;
    _residuals[0] = (T(getMeasurement()(0)) - _p[0]) / T(sqrt(getMeasurementCovariance()(0, 0)));
    _residuals[1] = (T(getMeasurement()(1)) - _p[1]) / T(sqrt(getMeasurementCovariance()(1, 1)));
    _residuals[2] = T(getMeasurement()(2)) - _o[0];
    //            std::cout << "+++++++  fix constraint +++++++" << std::endl;
    //            std::cout << "orientation:   " << _o[0] << std::endl;
    //            std::cout << "measurement:   " << T(getMeasurement()(2)) << std::endl;
    //            std::cout << "residual:      " << _residuals[2] << std::endl;
    //            std::cout << "is > PI        " << bool(_residuals[2] > T(2*M_PI)) << std::endl;
    //            std::cout << "is >= PI       " << bool(_residuals[2] <= T(-2*M_PI)) << std::endl;
    while (_residuals[2] > T(M_PI))
        _residuals[2] = _residuals[2] - T(2 * M_PI);
    while (_residuals[2] <= T(-M_PI))
        _residuals[2] = _residuals[2] + T(2 * M_PI);
    //            std::cout << "residual:      " << _residuals[2] << std::endl << std::endl;
    _residuals[2] = _residuals[2] / T(sqrt(getMeasurementCovariance()(2, 2)));
    //std::cout << "constraint fix computed!" << std::endl;
    return true;
}

} // namespace wolf

#endif
