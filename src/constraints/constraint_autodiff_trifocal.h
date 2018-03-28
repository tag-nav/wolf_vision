#ifndef _CONSTRAINT_AUTODIFF_TRIFOCAL_H_
#define _CONSTRAINT_AUTODIFF_TRIFOCAL_H_

//Wolf includes
//#include "wolf.h"
#include "constraint_autodiff.h"
#include "sensor_camera.h"

#include <vision_utils/common_class/trifocaltensor.h>

namespace wolf
{

WOLF_PTR_TYPEDEFS(ConstraintAutodiffTrifocal);

using namespace Eigen;

class ConstraintAutodiffTrifocal : public ConstraintAutodiff<ConstraintAutodiffTrifocal, 4, 3, 4, 3, 4, 3, 4, 3, 4>
{
        typedef ceres::Jet<Scalar, 28> JetType;
        typedef std::shared_ptr<vision_utils::TrifocalTensorBase<JetType> > TrifocalTensorJetPtr;
        typedef std::shared_ptr<vision_utils::TrifocalTensorBase<Scalar> >  TrifocalTensorScalarPtr;

    public:

        /** \brief Class constructor
         */
        ConstraintAutodiffTrifocal(const FeatureBasePtr& _feature_prev_ptr,
                                   const FeatureBasePtr& _feature_origin_ptr,
                                   const FeatureBasePtr& _feature_last_ptr,
                                   const ProcessorBasePtr& _processor_ptr,
                                   bool _apply_loss_function,
                                   ConstraintStatus _status);

        /** \brief Class Destructor
         */
        virtual ~ConstraintAutodiffTrifocal();

        /** brief : compute the residual from the state blocks being iterated by the solver.
         **/
        template<typename T>
        bool operator ()( const T* const _prev_pos,
                          const T* const _prev_quat,
                          const T* const _origin_pos,
                          const T* const _origin_quat,
                          const T* const _last_pos,
                          const T* const _last_quat,
                          const T* const _sen_pos,
                          const T* const _sen_quat,
                          T*             _residuals) const;

    private:
        template<typename D1, typename D2, typename D3>
        void expectation(MatrixBase<D1>     _wtr1,
                         QuaternionBase<D2> _wqr1,
                         MatrixBase<D1>     _wtr2,
                         QuaternionBase<D2> _wqr2,
                         MatrixBase<D1>     _wtr3,
                         QuaternionBase<D2> _wqr3,
                         MatrixBase<D1>     _rtc,
                         QuaternionBase<D2> _rqc,
                         MatrixBase<D3>     _expect) const;

        template<typename T>
        Matrix<T, 4, 1> residual(const Matrix<T, 9, 3> _expectation);

        template<typename D1, typename D2>
        void updateTensor(MatrixBase<D1>     _wtr1,
                           QuaternionBase<D2> _wqr1,
                           MatrixBase<D1>     _wtr2,
                           QuaternionBase<D2> _wqr2,
                           MatrixBase<D1>     _wtr3,
                           QuaternionBase<D2> _wqr3,
                           MatrixBase<D1>     _rtc,
                           QuaternionBase<D2> _rqc,
                           std::shared_ptr<vision_utils::TrifocalTensorBase<Scalar> > _tensor_ptr) const;

        template<typename D1, typename D2>
        void updateTensor(MatrixBase<D1>     _wtr1,
                           QuaternionBase<D2> _wqr1,
                           MatrixBase<D1>     _wtr2,
                           QuaternionBase<D2> _wqr2,
                           MatrixBase<D1>     _wtr3,
                           QuaternionBase<D2> _wqr3,
                           MatrixBase<D1>     _rtc,
                           QuaternionBase<D2> _rqc,
                           std::shared_ptr<vision_utils::TrifocalTensorBase<JetType> > _tensor_ptr) const;

        template<typename D1, typename D2>
        void computeTensor(MatrixBase<D1>     _wtr1,
                          QuaternionBase<D2> _wqr1,
                          MatrixBase<D1>     _wtr2,
                          QuaternionBase<D2> _wqr2,
                          MatrixBase<D1>     _wtr3,
                          QuaternionBase<D2> _wqr3,
                          MatrixBase<D1>     _rtc,
                          QuaternionBase<D2> _rqc,
                          std::shared_ptr<vision_utils::TrifocalTensorBase<typename D1::Scalar> > _tensor_ptr) const;



    private:
        FeatureBasePtr  feature_prev_ptr_;  // To look for measurements
        SensorCameraPtr camera_ptr_;        // To look for intrinsics
        Vector3s pixel_canonical_prev_, pixel_canonical_origin_, pixel_canonical_last_;
        TrifocalTensorScalarPtr tensor_scalar_ptr_;
        TrifocalTensorJetPtr    tensor_jet_ptr_;
        mutable bool first_Jet_;
};

} // namespace wolf

// Include here all headers for this class
//#include <YOUR_HEADERS.h>



namespace wolf
{

using namespace Eigen;

// Constructor
ConstraintAutodiffTrifocal::ConstraintAutodiffTrifocal(
        const FeatureBasePtr& _feature_prev_ptr,
        const FeatureBasePtr& _feature_origin_ptr,
        const FeatureBasePtr& _feature_last_ptr,
        const ProcessorBasePtr& _processor_ptr,
        bool _apply_loss_function,
        ConstraintStatus _status) : ConstraintAutodiff( CTR_TRIFOCAL_PLP,
                                                        _feature_origin_ptr->getFramePtr(),
                                                        _feature_origin_ptr->getCapturePtr(),
                                                        _feature_origin_ptr,
                                                        nullptr,
                                                        _processor_ptr,
                                                        _apply_loss_function,
                                                        _status,
                                                        _feature_prev_ptr->getFramePtr()->getPPtr(),
                                                        _feature_prev_ptr->getFramePtr()->getOPtr(),
                                                        _feature_origin_ptr->getFramePtr()->getPPtr(),
                                                        _feature_origin_ptr->getFramePtr()->getOPtr(),
                                                        _feature_last_ptr->getFramePtr()->getPPtr(),
                                                        _feature_last_ptr->getFramePtr()->getOPtr(),
                                                        _feature_last_ptr->getCapturePtr()->getSensorPPtr(),
                                                        _feature_last_ptr->getCapturePtr()->getSensorOPtr() ),
                                    feature_prev_ptr_(_feature_prev_ptr),
                                    camera_ptr_(std::static_pointer_cast<SensorCamera>(_processor_ptr->getSensorPtr())),
                                    tensor_scalar_ptr_(TrifocalTensorScalarPtr()),
                                    tensor_jet_ptr_(TrifocalTensorJetPtr()),
                                    first_Jet_(true)
{
    Matrix3s K_inv   = camera_ptr_->getIntrinsicMatrix().inverse();
    pixel_canonical_prev_   = K_inv * Vector3s(_feature_prev_ptr  ->getMeasurement(0), _feature_prev_ptr  ->getMeasurement(1), 1.0);
    pixel_canonical_origin_ = K_inv * Vector3s(_feature_origin_ptr->getMeasurement(0), _feature_origin_ptr->getMeasurement(1), 1.0);
    pixel_canonical_last_   = K_inv * Vector3s(_feature_last_ptr  ->getMeasurement(0), _feature_last_ptr  ->getMeasurement(1), 1.0);
}

// Destructor
ConstraintAutodiffTrifocal::~ConstraintAutodiffTrifocal()
{
}

template<typename T>
bool ConstraintAutodiffTrifocal::operator ()( const T* const _prev_pos, const T* const _prev_quat, const T* const _origin_pos, const T* const _origin_quat, const T* const _last_pos, const T* const _last_quat, const T* const _sen_pos, const T* const _sen_quat, T* _residuals) const
{

    // MAPS
    Map<const Matrix<T,3,1> > wtr1(_prev_pos);
    Map<const Quaternion<T> > wqr1(_prev_quat);
    Map<const Matrix<T,3,1> > wtr2(_origin_pos);
    Map<const Quaternion<T> > wqr2(_origin_quat);
    Map<const Matrix<T,3,1> > wtr3(_last_pos);
    Map<const Quaternion<T> > wqr3(_last_quat);
    Map<const Matrix<T,3,1> > rtc (_sen_pos);
    Map<const Quaternion<T> > rqc (_sen_quat);
    Map<Matrix<T,4,1> >      res (_residuals);

    Matrix<T, 9, 3> expect;
    expectation(wtr1, wqr1, wtr2, wqr2, wtr3, wqr3, rtc, rqc, expect);
    //    res = residual(expect);
    return true;
}

template<typename D1, typename D2, typename D3>
inline void ConstraintAutodiffTrifocal::expectation(MatrixBase<D1>     _wtr1,
                                                    QuaternionBase<D2> _wqr1,
                                                    MatrixBase<D1>     _wtr2,
                                                    QuaternionBase<D2> _wqr2,
                                                    MatrixBase<D1>     _wtr3,
                                                    QuaternionBase<D2> _wqr3,
                                                    MatrixBase<D1>     _rtc,
                                                    QuaternionBase<D2> _rqc,
                                                    MatrixBase<D3>     _expect) const
{
    std::shared_ptr<vision_utils::TrifocalTensorBase<typename D1::Scalar> > tensor_ptr;

    updateTensor(_wtr1, _wqr1, _wtr2, _wqr2, _wtr3, _wqr3, _rtc, _rqc, tensor_ptr);

}

template<typename D1, typename D2>
inline void ConstraintAutodiffTrifocal::updateTensor(
        MatrixBase<D1> _wtr1, QuaternionBase<D2> _wqr1, MatrixBase<D1> _wtr2, QuaternionBase<D2> _wqr2,
        MatrixBase<D1> _wtr3, QuaternionBase<D2> _wqr3, MatrixBase<D1> _rtc, QuaternionBase<D2> _rqc,
        std::shared_ptr<vision_utils::TrifocalTensorBase<Scalar> > _tensor_ptr) const
{
    computeTensor(_wtr1, _wqr1, _wtr2, _wqr2, _wtr3, _wqr3, _rtc, _rqc, _tensor_ptr);
    first_Jet_ = true;
}

template<typename D1, typename D2>
inline void ConstraintAutodiffTrifocal::updateTensor(
        MatrixBase<D1> _wtr1, QuaternionBase<D2> _wqr1, MatrixBase<D1> _wtr2, QuaternionBase<D2> _wqr2,
        MatrixBase<D1> _wtr3, QuaternionBase<D2> _wqr3, MatrixBase<D1> _rtc, QuaternionBase<D2> _rqc,
        std::shared_ptr<vision_utils::TrifocalTensorBase<JetType> > _tensor_ptr) const
{
    if (first_Jet_) // Only compute tensor the first time we need Jacobians.
    {
        computeTensor(_wtr1, _wqr1, _wtr2, _wqr2, _wtr3, _wqr3, _rtc, _rqc, tensor_jet_ptr_);
        first_Jet_ = false;
    }
    _tensor_ptr = tensor_jet_ptr_;
}

template<typename D1, typename D2>
inline void ConstraintAutodiffTrifocal::computeTensor(MatrixBase<D1> _wtr1, QuaternionBase<D2> _wqr1,
                                                      MatrixBase<D1> _wtr2, QuaternionBase<D2> _wqr2,
                                                      MatrixBase<D1> _wtr3, QuaternionBase<D2> _wqr3,
                                                      MatrixBase<D1> _rtc,  QuaternionBase<D2> _rqc,
                                                      std::shared_ptr<vision_utils::TrifocalTensorBase<typename D1::Scalar> > _tensor_ptr) const
{
    typedef typename D1::Scalar T;

    Matrix<T,3,3> wRc1, wRc2, wRc3;
    Matrix<T,3,1> wtc1, wtc2, wtc3;
    wtc1 = _wtr1 + _wqr1*_rtc;
    wtc2 = _wtr2 + _wqr2*_rtc;
    wtc3 = _wtr3 + _wqr3*_rtc;
    wRc1 = (_wqr1 * _rqc).matrix();
    wRc2 = (_wqr2 * _rqc).matrix();
    wRc3 = (_wqr3 * _rqc).matrix();

    // Relative transforms between cameras
    Matrix<T,3,1> c1tc2, c1tc3, c2tc3;
    Matrix<T,3,3> c1Rc2, c1Rc3, c2Rc3;
    c1tc2 = wRc1.transpose() * (wtc2 - wtc1);
    c1tc3 = wRc1.transpose() * (wtc3 - wtc1);
    c2tc3 = wRc2.transpose() * (wtc3 - wtc2);
    c1Rc2 = wRc1.transpose() * wRc2;
    c1Rc3 = wRc1.transpose() * wRc3;
    c2Rc3 = wRc2.transpose() * wRc3;
    //
    // Projection matrices (canonic cameras with origin in c1)
    Matrix<T,3,4> P2, P3;
    P2.block(0,0,3,3) = c1Rc2.transpose();
    P2.block(0,3,3,1) = -c1Rc2.transpose()*c1tc2;
    P3.block(0,0,3,3) = c1Rc3.transpose();
    P3.block(0,3,3,1) = -c1Rc3.transpose()*c1tc3;

    // build tensor
    _tensor_ptr->computeTensorFromProjectionMat(P2, P3);
}



}    // namespace wolf

#endif /* _CONSTRAINT_AUTODIFF_TRIFOCAL_H_ */
