#ifndef _CONSTRAINT_AUTODIFF_TRIFOCAL_H_
#define _CONSTRAINT_AUTODIFF_TRIFOCAL_H_

//Wolf includes
//#include "wolf.h"
#include "constraint_autodiff.h"
#include "sensor_camera.h"

#include <vision_utils/common_class/trifocaltensor.h>
#include <vision_utils/vision_utils.h>

namespace wolf
{

WOLF_PTR_TYPEDEFS(ConstraintAutodiffTrifocal);

using namespace Eigen;

class ConstraintAutodiffTrifocal : public ConstraintAutodiff<ConstraintAutodiffTrifocal, 3, 3, 4, 3, 4, 3, 4, 3, 4>
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

        const Vector3s& getPixelCanonicalLast() const
        {
            return pixel_canonical_last_;
        }

        void setPixelCanonicalLast(const Vector3s& pixelCanonicalLast)
        {
            pixel_canonical_last_ = pixelCanonicalLast;
        }

        const Vector3s& getPixelCanonicalOrigin() const
        {
            return pixel_canonical_origin_;
        }

        void setPixelCanonicalOrigin(const Vector3s& pixelCanonicalOrigin)
        {
            pixel_canonical_origin_ = pixelCanonicalOrigin;
        }

        const Vector3s& getPixelCanonicalPrev() const
        {
            return pixel_canonical_prev_;
        }

        void setPixelCanonicalPrev(const Vector3s& pixelCanonicalPrev)
        {
            pixel_canonical_prev_ = pixelCanonicalPrev;
        }

        const Matrix3s& getSqrtInformationUpper() const
        {
            return sqrt_information_upper;
        }

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
        template<typename D1, typename D2, class T, typename D3>
        void expectation(const MatrixBase<D1>&     _wtr1,
                         const QuaternionBase<D2>& _wqr1,
                         const MatrixBase<D1>&     _wtr2,
                         const QuaternionBase<D2>& _wqr2,
                         const MatrixBase<D1>&     _wtr3,
                         const QuaternionBase<D2>& _wqr3,
                         const MatrixBase<D1>&     _rtc,
                         const QuaternionBase<D2>& _rqc,
                         vision_utils::TrifocalTensorBase<T>& _tensor,
                         MatrixBase<D3>&           _c2Ec1) const;

        template<typename T, typename D1>
        Matrix<T, 3, 1> residual(const vision_utils::TrifocalTensorBase<T>& _tensor,
                                 const MatrixBase<D1>& _c2Ec1) const;

        // Helper functions to be used by the above
        template<class T, typename D1, typename D2, typename D3, typename D4>
        Matrix<T, 3, 1> error_jacobians(const vision_utils::TrifocalTensorBase<T>& _tensor,
                                        const MatrixBase<D1>& _c2Ec1,
                                        MatrixBase<D2>& _J_e_m1,
                                        MatrixBase<D3>& _J_e_m2,
                                        MatrixBase<D4>& _J_e_m3);


    private:
        FeatureBaseWPtr feature_prev_ptr_;  // To look for measurements
        SensorCameraPtr camera_ptr_;        // To look for intrinsics
        Vector3s pixel_canonical_prev_, pixel_canonical_origin_, pixel_canonical_last_;
        Matrix3s sqrt_information_upper;
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
                                                        nullptr,
                                                        nullptr,
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
                                    sqrt_information_upper(Matrix3s::Zero())
{
    setFeaturePtr(_feature_last_ptr);
    Matrix3s K_inv           = camera_ptr_->getIntrinsicMatrix().inverse();
    pixel_canonical_prev_    = K_inv * Vector3s(_feature_prev_ptr  ->getMeasurement(0), _feature_prev_ptr  ->getMeasurement(1), 1.0);
    pixel_canonical_origin_  = K_inv * Vector3s(_feature_origin_ptr->getMeasurement(0), _feature_origin_ptr->getMeasurement(1), 1.0);
    pixel_canonical_last_    = K_inv * Vector3s(_feature_last_ptr  ->getMeasurement(0), _feature_last_ptr  ->getMeasurement(1), 1.0);
    Matrix<Scalar,3,2> J_m_u = K_inv.block(0,0,3,2);

    // extract relevant states
    Vector3s    wtr1 =             _feature_prev_ptr  ->getFramePtr()  ->getPPtr()      ->getState();
    Quaternions wqr1 = Quaternions(_feature_prev_ptr  ->getFramePtr()  ->getOPtr()      ->getState().data() );
    Vector3s    wtr2 =             _feature_origin_ptr->getFramePtr()  ->getPPtr()      ->getState();
    Quaternions wqr2 = Quaternions(_feature_origin_ptr->getFramePtr()  ->getOPtr()      ->getState().data() );
    Vector3s    wtr3 =             _feature_last_ptr  ->getFramePtr()  ->getPPtr()      ->getState();
    Quaternions wqr3 = Quaternions(_feature_last_ptr  ->getFramePtr()  ->getOPtr()      ->getState().data() );
    Vector3s    rtc  =             _feature_last_ptr  ->getCapturePtr()->getSensorPPtr()->getState();
    Quaternions rqc  = Quaternions(_feature_last_ptr  ->getCapturePtr()->getSensorOPtr()->getState().data() );

    // expectation
    vision_utils::TrifocalTensorBase<Scalar> tensor;
    Matrix3s    c2Ec1;
    expectation(wtr1, wqr1,
                wtr2, wqr2,
                wtr3, wqr3,
                rtc, rqc,
                tensor, c2Ec1);

    // error Jacobians
    Matrix<Scalar,3,3> J_e_m1, J_e_m2, J_e_m3;
    error_jacobians(tensor, c2Ec1, J_e_m1, J_e_m2, J_e_m3);

    // chain rule to go from homogeneous pixel to Euclidean pixel
    Matrix<Scalar,3,2> J_e_u1 = J_e_m1 * J_m_u;
    Matrix<Scalar,3,2> J_e_u2 = J_e_m2 * J_m_u;
    Matrix<Scalar,3,2> J_e_u3 = J_e_m3 * J_m_u;

    // Error covariances induced by each of the measurement covariance
    Matrix3s Q1 = J_e_u1 * getFeaturePrevPtr() ->getMeasurementCovariance() * J_e_u1.transpose();
    Matrix3s Q2 = J_e_u2 * getFeatureOtherPtr()->getMeasurementCovariance() * J_e_u2.transpose();
    Matrix3s Q3 = J_e_u3 * getFeaturePtr()     ->getMeasurementCovariance() * J_e_u3.transpose();

    // Total error covariance
    Matrix3s Q = Q1 + Q2 + Q3;

    // Sqrt of information matrix
    Eigen::LLT<Eigen::MatrixXs> llt_of_info(Q.inverse()); // Cholesky decomposition
    sqrt_information_upper = llt_of_info.matrixU();
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
bool ConstraintAutodiffTrifocal::operator ()( const T* const _prev_pos,
                                              const T* const _prev_quat,
                                              const T* const _origin_pos,
                                              const T* const _origin_quat,
                                              const T* const _last_pos,
                                              const T* const _last_quat,
                                              const T* const _sen_pos,
                                              const T* const _sen_quat,
                                              T* _residuals) const
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
    Map<Matrix<T,3,1> >       res (_residuals);

    vision_utils::TrifocalTensorBase<T> tensor;
    Matrix<T, 3, 3> c2Ec1;
    expectation(wtr1, wqr1, wtr2, wqr2, wtr3, wqr3, rtc, rqc, tensor, c2Ec1);
    res = residual(tensor, c2Ec1);
    return true;
}

template<typename D1, typename D2, class T, typename D3>
inline void ConstraintAutodiffTrifocal::expectation(const MatrixBase<D1>&     _wtr1,
                                                    const QuaternionBase<D2>& _wqr1,
                                                    const MatrixBase<D1>&     _wtr2,
                                                    const QuaternionBase<D2>& _wqr2,
                                                    const MatrixBase<D1>&     _wtr3,
                                                    const QuaternionBase<D2>& _wqr3,
                                                    const MatrixBase<D1>&     _rtc,
                                                    const QuaternionBase<D2>& _rqc,
                                                    vision_utils::TrifocalTensorBase<T>& _tensor,
                                                    MatrixBase<D3>&     _c2Ec1) const
{

        typedef Translation<T, 3> TranslationType;
        typedef Eigen::Transform<T, 3, Eigen::Affine> TransformType;

        // All input Transforms
        TransformType wHr1 = TranslationType(_wtr1) * _wqr1;
        TransformType wHr2 = TranslationType(_wtr2) * _wqr2;
        TransformType wHr3 = TranslationType(_wtr3) * _wqr3;
        TransformType rHc  = TranslationType(_rtc)  * _rqc ;

        // Relative camera transforms
        TransformType c1Hc2 = rHc.inverse() * wHr1.inverse() * wHr2 * rHc;
        TransformType c1Hc3 = rHc.inverse() * wHr1.inverse() * wHr3 * rHc;

        // Projection matrices
        Matrix<T,3,4> c2Pc1 = c1Hc2.inverse().affine();
        Matrix<T,3,4> c3Pc1 = c1Hc3.inverse().affine();

        // Trifocal tensor
        _tensor.computeTensorFromProjectionMat(c2Pc1, c3Pc1);

        /* Essential matrix convention disambiguation
         *
         * C1 is the origin frame or reference
         * C2 is another cam
         * C2 is specified by R and T wrt C1 so that
         *   T is the position    of C2 wrt C1
         *   R is the orientation of C2 wrt C1
         * There is a 3D point P, noted P1 when expressed in C1 and P2 when expressed in C2:
         *   P1 = T + R * P2
         *
         * Coplanarity condition: a' * (b x c) = 0 with {a,b,c} three coplanar vectors.
         *
         * The three vectors are:
         *
         *   baseline: b  = T
         *   ray 1   : r1 = P1
         *   ray 2   : r2 = P1 - T = R*P2;
         *
         * so,
         *
         *   (r1)' * (b x r2) = 0 , which develops as:
         *
         *   P1' * (T x (R*P2))  = 0
         *   P1' * [T]x * R * P2 = 0
         *   P1' * c1Ec2 * P2    = 0 <--- Epipolar constraint
         *
         * therefore:
         *   c1Ec2 = [T]x * R        <--- Essential matrix
         *
         * or, if we prefer the constraint P2' * c2Ec1 * P1 = 0,
         *   c2Ec1 = c1Ec2' = R' * [T]x (we obviate the sign change)
         */
        _c2Ec1 =  c1Hc2.rotation().transpose() * skew(c1Hc2.translation()) ;
}

template<typename T, typename D1>
inline Matrix<T, 3, 1> ConstraintAutodiffTrifocal::residual(const vision_utils::TrifocalTensorBase<T>& _tensor,
                                                            const MatrixBase<D1>& _c2Ec1) const
{
    // 1. COMMON COMPUTATIONS

    // m1, m2, m3: canonical pixels in cams 1,2,3 -- canonical means m = K.inv * u, with _u_ a homogeneous pixel [ux; uy; 1].
    Matrix<T,3,1> m1(pixel_canonical_prev_  .cast<T>());
    Matrix<T,3,1> m2(pixel_canonical_origin_.cast<T>());
    Matrix<T,3,1> m3(pixel_canonical_last_  .cast<T>());

    // 2. TRILINEARITY PLP

    Matrix<T,2,1> e1;
    vision_utils::errorTrifocalPLP(m1, m2, m3, _tensor, _c2Ec1, e1);

    // 3. EPIPOLAR
    T e2;
    vision_utils::errorEpipolar(m1, m2, _c2Ec1, e2);

    // 4. RESIDUAL

    Matrix<T,3,1> errors, residual;
    errors  << e1, e2;
    residual = sqrt_information_upper.cast<T>() * errors;

    return residual;
}

// Helper functions to be used by the above
template<class T, typename D1, typename D2, typename D3, typename D4>
inline Matrix<T, 3, 1> ConstraintAutodiffTrifocal::error_jacobians(const vision_utils::TrifocalTensorBase<T>& _tensor,
                                                                   const MatrixBase<D1>& _c2Ec1,
                                                                   MatrixBase<D2>& _J_e_m1,
                                                                   MatrixBase<D3>& _J_e_m2,
                                                                   MatrixBase<D4>& _J_e_m3)
{
    // 1. COMMON COMPUTATIONS

    // m1, m2, m3: canonical pixels in cams 1,2,3 -- canonical means m = K.inv * u, with _u_ a homogeneous pixel [ux; uy; 1].
    Matrix<T,3,1> m1(pixel_canonical_prev_);
    Matrix<T,3,1> m2(pixel_canonical_origin_);
    Matrix<T,3,1> m3(pixel_canonical_last_);

    // 2. TRILINEARITY PLP

    Matrix<T,2,3> J_e1_m1, J_e1_m2, J_e1_m3;
    Matrix<T,2,1> e1;

    vision_utils::errorTrifocalPLP(m1, m2, m3, _tensor, _c2Ec1, e1, J_e1_m1, J_e1_m2, J_e1_m3);

    // 3. EPIPOLAR

    T e2;
    Matrix<T,1,3> J_e2_m1, J_e2_m2, J_e2_m3;

    vision_utils::errorEpipolar(m1, m2, _c2Ec1, e2, J_e2_m1, J_e2_m2);

    J_e2_m3.setZero(); // Not involved in epipolar c1->c2


    // Compact Jacobians
    _J_e_m1.topRows(2) = J_e1_m1;
    _J_e_m1.row(2)     = J_e2_m1;
    _J_e_m2.topRows(2) = J_e1_m2;
    _J_e_m2.row(2)     = J_e2_m2;
    _J_e_m3.topRows(2) = J_e1_m3;
    _J_e_m3.row(2)     = J_e2_m3;

    // 4. ERRORS

    Matrix<T,3,1> errors;
    errors  << e1, e2;

    return errors;

}


}    // namespace wolf


#endif /* _CONSTRAINT_AUTODIFF_TRIFOCAL_H_ */
