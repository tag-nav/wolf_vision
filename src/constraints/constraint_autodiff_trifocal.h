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
                         MatrixBase<D4>&           _C3Ec1) const;

        template<typename T, typename D1, typename D2>
        Matrix<T, 4, 1> residual(const vision_utils::TrifocalTensorBase<T>& _tensor,
                                 const MatrixBase<D1>& _c2Ec1,
                                 const MatrixBase<D2>& _C3Ec1);

        template<typename D1>
        Matrix<typename D1::Scalar, 2, 1> euclidean(const MatrixBase<D1>& _homog)
        {
            Matrix<typename D1::Scalar, 2, 1> euc = _homog.head(2) / _homog(2);
        }

        template<typename D1, typename D2>
        Matrix<typename D1::Scalar, 2, 1> euclidean(const MatrixBase<D1>& _homog, MatrixBase<D2>& _J_e_h)
        {
            Matrix<typename D1::Scalar, 2, 1> euc = euclidean(_homog);
            _J_e_h.block(0,0,2,2) = Matrix<typename D1::Scalar,2,2>::Identity() / _homog(2);
            _J_e_h.block(0,2,2,1) = - euc / _homog(2);
        }

        template<typename D1, typename D2, typename D3, typename D4>
        typename D1::Scalar distancePointLine(const MatrixBase<D1>& _point, const MatrixBase<D2>& _line, MatrixBase<D3>& _J_d_p, MatrixBase<D4>& _J_d_l )
        {
            typedef typename D1::Scalar T;

            T nn2   = _line.head(2).squaredNorm();
            T nn    = sqrt(nn2);
            T nn3   = nn2 * nn;
            T ltp   = _line.dot(_point);
            T p2nn  = _point(2) * nn;
            T p2nn3 = _point(2) * nn3;

            Matrix<T,2,1> u = _point.head(2)/_point(2); // euclidean point
            Matrix<T,2,1> l = _line .head(2)/_point(2); // director vector

            T d     = ltp / p2nn;

            _J_d_p << l(0)/nn               , l(1)/nn               ,  l(2)/nn - ltp/(_point(2)*_point(2)*nn);

            _J_d_l << u(0)/nn - ltp*l(0)/nn3, u(1)/nn - ltp*l(1)/nn3,  1/nn                                  ;

            return d;
        }

    private:
        FeatureBasePtr  feature_prev_ptr_;  // To look for measurements
        SensorCameraPtr camera_ptr_;        // To look for intrinsics
        Vector3s pixel_canonical_prev_, pixel_canonical_origin_, pixel_canonical_last_;
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
                                    camera_ptr_(std::static_pointer_cast<SensorCamera>(_processor_ptr->getSensorPtr()))
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
                                                    MatrixBase<D4>&     _C3Ec1) const
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
    vision_utils::TrifocalTensorBase<T> tensor;
    tensor.computeTensorFromProjectionMat(P2, P3);

    // compute essential matrices c2c1 and c3c2
    Eigen::Matrix<T, 3, 3> c2Ec1, c3Ec2;
    c2Ec1 = c1Rc2.transpose()*skew(c1tc2);
    c3Ec2 = c2Rc3.transpose()*skew(c2tc3);
}

template<typename T, typename D1, typename D2>
inline Matrix<T, 4, 1> ConstraintAutodiffTrifocal::residual(const vision_utils::TrifocalTensorBase<T>& _tensor,
                                                            const MatrixBase<D1>& _c2Ec1,
                                                            const MatrixBase<D2>& _c3Ec1)
{
    Matrix<T,3,1> m1(pixel_canonical_prev_);
    Matrix<T,3,1> m2(pixel_canonical_origin_);
    Matrix<T,3,1> m3(pixel_canonical_last_);

    // l2 and l3
    Matrix<T,3,1> l2        = _c2Ec1*m1;
    Matrix<T,3,3> J_l2_m1   = _c2Ec1;
    Matrix<T,3,1> l3        = _c3Ec1*m1;
    Matrix<T,3,3> J_l3_m1   = _c3Ec1;

    // p2 line in cam 2 perpendicular to epipolar
    Matrix<T,3,1> p2;
    p2(0) = l2(1);
    p2(1) = -l2(0);
    p2(2) = -m2(0)*l2(1) + m2(1)*l2(0);
    Matrix<T,3,3> J_p2_m2 = (Matrix<T,3,3>() << (T)0, (T)0, (T)0, (T)0, (T)0, (T)0, -l2(1), l2(0), (T)0).finished();
    Matrix<T,3,3> J_p2_l2 = (Matrix<T,3,3>() << (T)0, (T)1, (T)0, (T)-1, (T)0, (T)0, m2(1), -m2(0), (T)0).finished();

    // Tensor slices
    Matrix<T,3,3> T0, T1, T2;
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
    Matrix<T,3,1> m3e;      m3e      << p2(0) * T0m1 , p2(1) * T1m1 , p2(2) * T2m1;
    Matrix<T,3,3> J_m3e_p2; J_m3e_p2 << T0m1, T1m1, T2m1;
    T J_m3e_T0m1 = p2(0); // scalar times identity
    T J_m3e_T1m1 = p2(1); // scalar times identity
    T J_m3e_T2m1 = p2(2); // scalar times identity

    // Go to Euclidean plane
    Matrix<T,2,3> J_u3e_m3e, J_u3_m3;
    Matrix<T,2,1> u3e = euclidean(m3e, J_u3e_m3e);
    Matrix<T,2,1> u3  = euclidean(m3, J_u3_m3);

    Matrix<T,2,1> e1   = u3 - u3e;
    T J_e1_u3  = (T)1;  // scalar times identity
    T J_e1_u3e = (T)-1; // scalar times identity

    // chain rule
    Matrix<T,3,3> J_e1_m3e = J_e1_u3e * J_u3e_m3e;
    Matrix<T,3,3> J_e1_m1 =
            J_e1_m3e * J_m3e_T0m1 * J_T0m1_m1 +
            J_e1_m3e * J_m3e_T1m1 * J_T1m1_m1 +
            J_e1_m3e * J_m3e_T2m1 * J_T2m1_m1 +
            J_e1_m3e * J_m3e_p2   * J_p2_l2   * J_l2_m1;
    Matrix<T,3,3> J_e1_m2 = J_e1_m3e * J_m3e_p2 * J_p2_m2;
    Matrix<T,3,3> J_e1_m3 = J_e1_u3  * J_u3_m3;

    // covariances
    Matrix<T,2,2> Q1 = J_e1_m1 * feature_prev_ptr_->getMeasurementCovariance()         * J_e1_m1.transpose()
                    + J_e1_m2  * feature_other_ptr_.lock()->getMeasurementCovariance() * J_e1_m2.transpose()
                    + J_e1_m3  * feature_ptr_.lock()->getMeasurementCovariance()       * J_e1_m3.transpose();



    /////// epipolars
    Matrix<T,1,3> J_e2_m2, J_e2_l2;
    Matrix<T,1,3> J_e3_m3, J_e3_l3;
    T e2 = distancePointLine(m2, l2, J_e2_m2, J_e2_l2);
    T e3 = distancePointLine(m3, l3, J_e3_m3, J_e3_l3);

    // chain rule
    Matrix<T,1,3> J_e2_m1 = J_e2_l2 * J_l2_m1;
    Matrix<T,1,3> J_e3_m1 = J_e3_l3 * J_l3_m1;

    Matrix<T,1,1> Q2 =        J_e2_m1 * feature_prev_ptr_->getMeasurementCovariance() * J_e2_m1.transpose()
                            + J_e2_m2 * feature_other_ptr_.lock()->getMeasurementCovariance() * J_e2_m2.transpose();

    Matrix<T,1,1> Q3 =        J_e3_m1 * feature_prev_ptr_->getMeasurement() * J_e3_m1.transpose()
                            + J_e3_m3 * feature_ptr_.lock()->getMeasurementCovariance() * J_e3_m3.transpose();

    // sqrt info
    Eigen::LLT<Eigen::MatrixXs> llt_of_info(Q1.inverse()); // Cholesky decomposition
    Matrix<T,2,2> W1 = llt_of_info.matrixU();
    T w2             = 1 / sqrt(Q2(0));
    T w3             = 1 / sqrt(Q3(0));

    // residuals
    Matrix<T,4,1> residual;
    residual.head(2) = W1 * e1;
    residual(2)      = w2 * e2;
    residual(3)      = w3 * e3;

    return residual;
}



}    // namespace wolf


#endif /* _CONSTRAINT_AUTODIFF_TRIFOCAL_H_ */
