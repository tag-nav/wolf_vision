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

        FeatureBasePtr getFeaturePrevPtr();

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

    public:
        template<typename D1, typename D2, typename D3, typename D4>
        void expectation(const MatrixBase<D1>&     _wtr1,
                         const QuaternionBase<D2>& _wqr1,
                         const MatrixBase<D1>&     _wtr2,
                         const QuaternionBase<D2>& _wqr2,
                         const MatrixBase<D1>&     _wtr3,
                         const QuaternionBase<D2>& _wqr3,
                         const MatrixBase<D1>&     _rtc,
                         const QuaternionBase<D2>& _rqc,
                         vision_utils::TrifocalTensorBase<typename D1::Scalar>& _tensor,
                         MatrixBase<D3>&           _c2Ec1,
                         MatrixBase<D4>&           _c3Ec1) const;

        template<typename T, typename D1, typename D2>
        Matrix<T, 4, 1> residual(const vision_utils::TrifocalTensorBase<T>& _tensor,
                                 const MatrixBase<D1>& _c2Ec1,
                                 const MatrixBase<D2>& _c3Ec1) const;

    private:
        // Helper functions to be used by the above
        template<class TT, typename D1, typename D2, typename D3, typename D4, typename D5, typename D6, typename D7, typename D8, typename D9, typename D10, typename D11>
        void residual_jacobians(const vision_utils::TrifocalTensorBase<TT>& _tensor,
                      const MatrixBase<D1>& _c2Ec1,
                      const MatrixBase<D2>& _c3Ec1,
                      MatrixBase<D3>& _J_e1_m1,
                      MatrixBase<D4>& _J_e1_m2,
                      MatrixBase<D5>& _J_e1_m3,
                      MatrixBase<D6>& _J_e2_m1,
                      MatrixBase<D7>& _J_e2_m2,
                      MatrixBase<D8>& _J_e2_m3,
                      MatrixBase<D9>& _J_e3_m1,
                      MatrixBase<D10>& _J_e3_m2,
                      MatrixBase<D11>& _J_e3_m3);


    private:
        FeatureBaseWPtr feature_prev_ptr_;  // To look for measurements
        SensorCameraPtr camera_ptr_;        // To look for intrinsics
        Vector3s pixel_canonical_prev_, pixel_canonical_origin_, pixel_canonical_last_;
        Matrix4s sqrt_information_upper;
};

} // namespace wolf



// Includes for implentation
#include "rotations.h"

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
                                    sqrt_information_upper(Matrix4s::Zero())
{
    setFeaturePtr(_feature_last_ptr);
    Matrix3s K_inv   = camera_ptr_->getIntrinsicMatrix().inverse();
    pixel_canonical_prev_   = K_inv * Vector3s(_feature_prev_ptr  ->getMeasurement(0), _feature_prev_ptr  ->getMeasurement(1), 1.0);
    pixel_canonical_origin_ = K_inv * Vector3s(_feature_origin_ptr->getMeasurement(0), _feature_origin_ptr->getMeasurement(1), 1.0);
    pixel_canonical_last_   = K_inv * Vector3s(_feature_last_ptr  ->getMeasurement(0), _feature_last_ptr  ->getMeasurement(1), 1.0);
    Matrix<Scalar,3,2> J_m_u;
    J_m_u << K_inv.block(0,0,3,2);

    // extract relevant states
    Vector3s    wtr1 =             _feature_prev_ptr->getFramePtr()->getPPtr()->getState();
    Quaternions wqr1 = Quaternions(_feature_prev_ptr->getFramePtr()->getOPtr()->getState().data() );
    Vector3s    wtr2 =             _feature_origin_ptr->getFramePtr()->getPPtr()->getState();
    Quaternions wqr2 = Quaternions(_feature_origin_ptr->getFramePtr()->getOPtr()->getState().data() );
    Vector3s    wtr3 =             _feature_last_ptr->getFramePtr()->getPPtr()->getState();
    Quaternions wqr3 = Quaternions(_feature_last_ptr->getFramePtr()->getOPtr()->getState().data() );
    Vector3s    rtc  =             _feature_last_ptr->getCapturePtr()->getSensorPPtr()->getState();
    Quaternions rqc  = Quaternions(_feature_last_ptr->getCapturePtr()->getSensorOPtr()->getState().data() );
    vision_utils::TrifocalTensorBase<Scalar> tensor;
    Matrix3s    c2Ec1, c3Ec1;

    // expectation
    expectation(wtr1, wqr1, wtr2, wqr2, wtr3, wqr3, rtc, rqc, tensor, c2Ec1, c3Ec1);

    // residual and Jacobians
    Matrix<Scalar,2,3> J_e1_m1, J_e1_m2, J_e1_m3;
    Matrix<Scalar,1,3> J_e2_m1, J_e2_m2, J_e2_m3, J_e3_m1, J_e3_m2, J_e3_m3;
    residual_jacobians(tensor, c2Ec1, c3Ec1, J_e1_m1, J_e1_m2, J_e1_m3, J_e2_m1, J_e2_m2, J_e2_m3, J_e3_m1, J_e3_m2, J_e3_m3);

    // chain rule
    Matrix2s J_e1_u1 = J_e1_m1 * J_m_u;
    Matrix2s J_e1_u2 = J_e1_m2 * J_m_u;
    Matrix2s J_e1_u3 = J_e1_m3 * J_m_u;
    Matrix<Scalar,1,2> J_e2_u1 = J_e2_m1 * J_m_u;
    Matrix<Scalar,1,2> J_e2_u2 = J_e2_m2 * J_m_u;
    Matrix<Scalar,1,2> J_e3_u1 = J_e3_m1 * J_m_u;
    Matrix<Scalar,1,2> J_e3_u3 = J_e3_m3 * J_m_u;

    // Covariances
    Matrix2s Q1 = J_e1_u1 * getFeaturePrevPtr()->getMeasurementCovariance()  * J_e1_u1.transpose()
               + J_e1_u2  * getFeatureOtherPtr()->getMeasurementCovariance() * J_e1_u2.transpose()
               + J_e1_u3  * getFeaturePtr()->getMeasurementCovariance()      * J_e1_u3.transpose();

    Matrix1s Q2 = J_e2_u1 * getFeaturePrevPtr()->getMeasurementCovariance()  * J_e2_u1.transpose()
                + J_e2_u2 * getFeatureOtherPtr()->getMeasurementCovariance() * J_e2_u2.transpose();

    Matrix1s Q3 = J_e3_u1 * getFeaturePrevPtr()->getMeasurementCovariance()  * J_e3_u1.transpose()
                + J_e3_u3 * getFeaturePtr()->getMeasurementCovariance()      * J_e3_u3.transpose();

    // sqrt info
    Eigen::LLT<Eigen::MatrixXs> llt_of_info(Q1.inverse()); // Cholesky decomposition
    sqrt_information_upper.block(0,0,2,2) = llt_of_info.matrixU();
    sqrt_information_upper(2,2)           = 1 / sqrt(Q2(0));
    sqrt_information_upper(3,3)           = 1 / sqrt(Q3(0));

}

// Destructor
ConstraintAutodiffTrifocal::~ConstraintAutodiffTrifocal()
{
}

inline FeatureBasePtr ConstraintAutodiffTrifocal::getFeaturePrevPtr()
{
    return feature_prev_ptr_.lock();
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

    vision_utils::TrifocalTensorBase<T> tensor;
    Matrix<T, 3, 3> c2Ec1, c3Ec1;
    expectation(wtr1, wqr1, wtr2, wqr2, wtr3, wqr3, rtc, rqc, tensor, c2Ec1, c3Ec1);
    //    res = residual(expect);
    return true;
}

template<typename D1, typename D2, typename D3, typename D4>
inline void ConstraintAutodiffTrifocal::expectation(const MatrixBase<D1>&     _wtr1,
                                                    const QuaternionBase<D2>& _wqr1,
                                                    const MatrixBase<D1>&     _wtr2,
                                                    const QuaternionBase<D2>& _wqr2,
                                                    const MatrixBase<D1>&     _wtr3,
                                                    const QuaternionBase<D2>& _wqr3,
                                                    const MatrixBase<D1>&     _rtc,
                                                    const QuaternionBase<D2>& _rqc,
                                                    vision_utils::TrifocalTensorBase<typename D1::Scalar>& _tensor,
                                                    MatrixBase<D3>&     _c2Ec1,
                                                    MatrixBase<D4>&     _c3Ec1) const
{
    // compute tensor
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

    // Projection matrices (canonic cameras with origin in c1)
    Matrix<T,3,4> P2, P3;
    P2.block(0,0,3,3) = c1Rc2.transpose();
    P2.block(0,3,3,1) = -c1Rc2.transpose()*c1tc2;
    P3.block(0,0,3,3) = c1Rc3.transpose();
    P3.block(0,3,3,1) = -c1Rc3.transpose()*c1tc3;

    // compute tensor
    _tensor.computeTensorFromProjectionMat(P2, P3);

    // compute essential matrices c2c1 and c3c1
    _c2Ec1 = c1Rc2.transpose()*skew(c1tc2);
    _c3Ec1 = c1Rc3.transpose()*skew(c1tc3);
}

template<typename T, typename D1, typename D2>
inline Matrix<T, 4, 1> ConstraintAutodiffTrifocal::residual(const vision_utils::TrifocalTensorBase<T>& _tensor,
                                                            const MatrixBase<D1>& _c2Ec1,
                                                            const MatrixBase<D2>& _c3Ec1) const
{
    // m1, m2, m3: canonical pixels in cams 1,2,3 -- canonical means m = K.inv * u, with _u_ a homogeneous pixel [ux; uy; 1].
    Matrix<T,3,1> m1(pixel_canonical_prev_);
    Matrix<T,3,1> m2(pixel_canonical_origin_);
    Matrix<T,3,1> m3(pixel_canonical_last_);

    // l2 and l3: epipolar lines of m1 in cam 2 and cam 3
    Matrix<T,3,1> l2        = _c2Ec1*m1;
    Matrix<T,3,1> l3        = _c3Ec1*m1;

    // p2: line in cam 2 perpendicular to epipolar
    Matrix<T,3,1> p2;
    p2(0) = l2(1);
    p2(1) = -l2(0);
    p2(2) = -m2(0)*l2(1) + m2(1)*l2(0);

    // Tensor slices
    Matrix<T,3,3> T0, T1, T2;
    _tensor.getLayers(T0, T1, T2);

    // freedom for catalonia
    Matrix<T,3,1> T0m1, T1m1, T2m1;
    T0m1 = T0*m1;
    T1m1 = T1*m1;
    T2m1 = T2*m1;

    // PLP trilinearity error
    Matrix<T,3,1> m3e;
    m3e      << p2(0) * T0m1 , p2(1) * T1m1 , p2(2) * T2m1;

    T J_m3e_T0m1 = p2(0); // scalar times identity
    T J_m3e_T1m1 = p2(1); // scalar times identity
    T J_m3e_T2m1 = p2(2); // scalar times identity

    // Go to Euclidean plane
    Matrix<T,2,1> u3e = vision_utils::euclidean(m3e);
    Matrix<T,2,1> u3  = vision_utils::euclidean(m3);

    Matrix<T,2,1> e1   = u3 - u3e;

    /////// epipolars
    T e2 = vision_utils::distancePointLine(m2, l2);
    T e3 = vision_utils::distancePointLine(m3, l3);

    // residuals
    Matrix<T,4,1> errors, residual;
    errors << e1, e2, e3;
    residual = sqrt_information_upper * errors;

    return residual;
}

// Helper functions to be used by the above
template<class TT, typename D1, typename D2, typename D3, typename D4, typename D5, typename D6, typename D7, typename D8, typename D9, typename D10, typename D11>
inline void ConstraintAutodiffTrifocal::residual_jacobians(const vision_utils::TrifocalTensorBase<TT>& _tensor,
                                                           const MatrixBase<D1>& _c2Ec1,
                                                           const MatrixBase<D2>& _c3Ec1,
                                                           MatrixBase<D3>& _J_e1_m1,
                                                           MatrixBase<D4>& _J_e1_m2,
                                                           MatrixBase<D5>& _J_e1_m3,
                                                           MatrixBase<D6>& _J_e2_m1,
                                                           MatrixBase<D7>& _J_e2_m2,
                                                           MatrixBase<D8>& _J_e2_m3,
                                                           MatrixBase<D9>& _J_e3_m1,
                                                           MatrixBase<D10>& _J_e3_m2,
                                                           MatrixBase<D11>& _J_e3_m3)
{
    typedef typename D1::Scalar T;

    // 1. COMMON COMPUTATIONS

    // m1, m2, m3: canonical pixels in cams 1,2,3 -- canonical means m = K.inv * u, with _u_ a homogeneous pixel [ux; uy; 1].
    Matrix<T,3,1> m1(pixel_canonical_prev_);
    Matrix<T,3,1> m2(pixel_canonical_origin_);
    Matrix<T,3,1> m3(pixel_canonical_last_);

    // l2 and l3: epipolar lines of m1 in cam 2 and cam 3
    Matrix<T,3,1> l2        = _c2Ec1*m1;
    Matrix<T,3,3> J_l2_m1   = _c2Ec1;
    Matrix<T,3,1> l3        = _c3Ec1*m1;
    Matrix<T,3,3> J_l3_m1   = _c3Ec1;


    // 2. TRILINEARITY PLP

    // p2: line in cam 2 perpendicular to epipolar
    Matrix<T,3,1> p2;
    p2(0) = l2(1);
    p2(1) = -l2(0);
    p2(2) = -m2(0)*l2(1) + m2(1)*l2(0);
    Matrix<T,3,3> J_p2_m2 = (Matrix<T,3,3>() << (T)0, (T)0, (T)0, (T)0, (T)0, (T)0, -l2(1), l2(0), (T)0).finished();
    Matrix<T,3,3> J_p2_l2 = (Matrix<T,3,3>() << (T)0, (T)1, (T)0, (T)-1, (T)0, (T)0, m2(1), -m2(0), (T)0).finished();

    // Tensor slices
    Matrix<TT,3,3> T0, T1, T2;
    _tensor.getLayers(T0, T1, T2);

    // freedom for catalonia
    Matrix<T,3,1> T0m1, T1m1, T2m1;
    T0m1 = T0*m1;
    T1m1 = T1*m1;
    T2m1 = T2*m1;

    Matrix<T,3,3> J_T0m1_m1 = T0;
    Matrix<T,3,3> J_T1m1_m1 = T1;
    Matrix<T,3,3> J_T2m1_m1 = T2;

    // PLP trilinearity error
    Matrix<T,3,1> m3e;
    Matrix<T,3,3> J_m3e_p2;
    m3e      =  p2(0) * T0m1 + p2(1) * T1m1 + p2(2) * T2m1;
    J_m3e_p2 << T0m1, T1m1, T2m1;

    T J_m3e_T0m1 = p2(0); // scalar times identity
    T J_m3e_T1m1 = p2(1); // scalar times identity
    T J_m3e_T2m1 = p2(2); // scalar times identity

    // Go to Euclidean plane
    Matrix<T,2,3> J_u3e_m3e, J_u3_m3;
    // Matrix<T,2,1> u3e = vision_utils::euclidean(m3e, J_u3e_m3e);
    // Matrix<T,2,1> u3  = vision_utils::euclidean(m3, J_u3_m3);
    vision_utils::euclidean(m3e, J_u3e_m3e);
    vision_utils::euclidean(m3, J_u3_m3);

    // compute trilinearity PLP error
    // Matrix<T,2,1> e1   = u3 - u3e;
    T J_e1_u3  = (T)1;  // scalar times identity
    T J_e1_u3e = (T)-1; // scalar times identity

    // Jacobians of PLP by the chain rule
    Matrix<T,2,3> J_e1_m3e = J_e1_u3e * J_u3e_m3e;
    _J_e1_m1 =
            J_e1_m3e * J_m3e_T0m1 * J_T0m1_m1 +
            J_e1_m3e * J_m3e_T1m1 * J_T1m1_m1 +
            J_e1_m3e * J_m3e_T2m1 * J_T2m1_m1 +
            J_e1_m3e * J_m3e_p2   * J_p2_l2   * J_l2_m1;
    _J_e1_m2 = J_e1_m3e * J_m3e_p2 * J_p2_m2;
    _J_e1_m3 = J_e1_u3  * J_u3_m3;


    // 3. EPIPOLARS

    Matrix<T,1,3> J_e2_l2;
    Matrix<T,1,3> J_e3_l3;
    // T e2 = vision_utils::distancePointLine(m2, l2, J_e2_m2, J_e2_l2);
    // T e3 = vision_utils::distancePointLine(m3, l3, J_e3_m3, J_e3_l3);
    vision_utils::distancePointLine(m2, l2, _J_e2_m2, J_e2_l2);
    vision_utils::distancePointLine(m3, l3, _J_e3_m3, J_e3_l3);

    // chain rule
    _J_e2_m1 = J_e2_l2 * J_l2_m1;
    _J_e3_m1 = J_e3_l3 * J_l3_m1;
}


}    // namespace wolf


#endif /* _CONSTRAINT_AUTODIFF_TRIFOCAL_H_ */
