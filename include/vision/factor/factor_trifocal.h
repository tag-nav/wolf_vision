#ifndef _FACTOR_TRIFOCAL_H_
#define _FACTOR_TRIFOCAL_H_

//Wolf includes
//#include "core/common/wolf.h"
#include "core/factor/factor_autodiff.h"
#include "vision/sensor/sensor_camera.h"

#include <vision_utils/common_class/trifocaltensor.h>
#include <vision_utils/vision_utils.h>

namespace wolf
{

WOLF_PTR_TYPEDEFS(FactorTrifocal);

using namespace Eigen;

class FactorTrifocal : public FactorAutodiff<FactorTrifocal, 3, 3, 4, 3, 4, 3, 4, 3, 4>
{
    public:

        /** \brief Class constructor
         */
        FactorTrifocal(const FeatureBasePtr& _feature_1_ptr,
                               const FeatureBasePtr& _feature_2_ptr,
                               const FeatureBasePtr& _feature_own_ptr,
                               const ProcessorBasePtr& _processor_ptr,
                               bool _apply_loss_function,
                               FactorStatus _status);

        /** \brief Class Destructor
         */
        ~FactorTrifocal() override;

        std::string getTopology() const override
        {
            return std::string("GEOM");
        }

        FeatureBasePtr getFeaturePrev();

        const Vector3d& getPixelCanonical3() const
        {
            return pixel_canonical_3_;
        }

        void setPixelCanonical3(const Vector3d& pixelCanonical3)
        {
            pixel_canonical_3_ = pixelCanonical3;
        }

        const Vector3d& getPixelCanonical2() const
        {
            return pixel_canonical_2_;
        }

        void setPixelCanonical2(const Vector3d& pixelCanonical2)
        {
            pixel_canonical_2_ = pixelCanonical2;
        }

        const Vector3d& getPixelCanonical1() const
        {
            return pixel_canonical_1_;
        }

        void setPixelCanonical1(const Vector3d& pixelCanonical1)
        {
            pixel_canonical_1_ = pixelCanonical1;
        }

        const Matrix3d& getSqrtInformationUpper() const
        {
            return sqrt_information_upper;
        }

        /** brief : compute the residual from the state blocks being iterated by the solver.
         **/
        template<typename T>
        bool operator ()( const T* const _pos1,
                          const T* const _quat1,
                          const T* const _pos2,
                          const T* const _quat2,
                          const T* const _pos3,
                          const T* const _quat3,
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
        Vector3d pixel_canonical_1_, pixel_canonical_2_, pixel_canonical_3_;

        Matrix3d sqrt_information_upper;

        //Print function specialized for doubles (avoid jets)
        template <class T, int ROWS, int COLS>
        void print_matrix(const Eigen::Matrix<T, ROWS, COLS>& _mat) const;

        template <int ROWS, int COLS>
        void print_matrix(const Eigen::Matrix<double, ROWS, COLS>& _mat) const;

        template<class T>
        void print_scalar(const T& _val) const;

        void print_scalar(const double& _val) const;
};

} // namespace wolf

// Includes for implentation
#include "core/math/rotations.h"

namespace wolf
{

using namespace Eigen;

// Constructor
FactorTrifocal::FactorTrifocal(const FeatureBasePtr& _feature_1_ptr,
                               const FeatureBasePtr& _feature_2_ptr,
                               const FeatureBasePtr& _feature_own_ptr,
                               const ProcessorBasePtr& _processor_ptr,
                               bool _apply_loss_function,
                               FactorStatus _status) :
        FactorAutodiff( "TRIFOCAL PLP",
                        _feature_own_ptr,
                        nullptr,
                        nullptr,
                        _feature_2_ptr, //< this sets feature 2 (the one between the oldest and the newest)
                        nullptr,
                        _processor_ptr,
                        _apply_loss_function,
                        _status,
                        _feature_1_ptr->getFrame()->getP(),
                        _feature_1_ptr->getFrame()->getO(),
                        _feature_2_ptr->getFrame()->getP(),
                        _feature_2_ptr->getFrame()->getO(),
                        _feature_own_ptr->getFrame()->getP(),
                        _feature_own_ptr->getFrame()->getO(),
                        _feature_own_ptr->getCapture()->getSensorP(),
                        _feature_own_ptr->getCapture()->getSensorO() ),
        feature_prev_ptr_(_feature_1_ptr),
        camera_ptr_(std::static_pointer_cast<SensorCamera>(_processor_ptr->getSensor())),
        sqrt_information_upper(Matrix3d::Zero())
{
    Matrix3d K_inv           = camera_ptr_->getIntrinsicMatrix().inverse();
    pixel_canonical_1_      = K_inv * Vector3d(_feature_1_ptr->getMeasurement(0), _feature_1_ptr->getMeasurement(1), 1.0);
    pixel_canonical_2_      = K_inv * Vector3d(_feature_2_ptr->getMeasurement(0), _feature_2_ptr->getMeasurement(1), 1.0);
    pixel_canonical_3_      = K_inv * Vector3d(_feature_own_ptr->getMeasurement(0), _feature_own_ptr->getMeasurement(1), 1.0);
    Matrix<double,3,2> J_m_u = K_inv.block(0,0,3,2);

    // extract relevant states
    Vector3d    wtr1 =             _feature_1_ptr  ->getFrame()  ->getP()      ->getState();
    Quaterniond wqr1 = Quaterniond(_feature_1_ptr  ->getFrame()  ->getO()      ->getState().data() );
    Vector3d    wtr2 =             _feature_2_ptr  ->getFrame()  ->getP()      ->getState();
    Quaterniond wqr2 = Quaterniond(_feature_2_ptr  ->getFrame()  ->getO()      ->getState().data() );
    Vector3d    wtr3 =             _feature_own_ptr->getFrame()  ->getP()      ->getState();
    Quaterniond wqr3 = Quaterniond(_feature_own_ptr->getFrame()  ->getO()      ->getState().data() );
    Vector3d    rtc  =             _feature_own_ptr->getCapture()->getSensorP()->getState();
    Quaterniond rqc  = Quaterniond(_feature_own_ptr->getCapture()->getSensorO()->getState().data() );

    // expectation // canonical units
    vision_utils::TrifocalTensorBase<double> tensor;
    Matrix3d    c2Ec1;
    expectation(wtr1, wqr1,
                wtr2, wqr2,
                wtr3, wqr3,
                rtc, rqc,
                tensor, c2Ec1);

    // error Jacobians // canonical units
    Matrix<double,3,3> J_e_m1, J_e_m2, J_e_m3;
    error_jacobians(tensor, c2Ec1, J_e_m1, J_e_m2, J_e_m3);

    // chain rule to go from homogeneous pixel to Euclidean pixel
    Matrix<double,3,2> J_e_u1 = J_e_m1 * J_m_u;
    Matrix<double,3,2> J_e_u2 = J_e_m2 * J_m_u;
    Matrix<double,3,2> J_e_u3 = J_e_m3 * J_m_u;

    // Error covariances induced by each of the measurement covariance // canonical units
    Matrix3d Q1 = J_e_u1 * _feature_1_ptr   ->getMeasurementCovariance() * J_e_u1.transpose();
    Matrix3d Q2 = J_e_u2 * _feature_2_ptr   ->getMeasurementCovariance() * J_e_u2.transpose();
    Matrix3d Q3 = J_e_u3 * _feature_own_ptr ->getMeasurementCovariance() * J_e_u3.transpose();

    // Total error covariance // canonical units
    Matrix3d Q = Q1 + Q2 + Q3;

    // Sqrt of information matrix // canonical units
    Eigen::LLT<Eigen::MatrixXd> llt_of_info(Q.inverse()); // Cholesky decomposition
    sqrt_information_upper = llt_of_info.matrixU();

    // Re-write info matrix (for debug only)
    //    double pix_noise = 1.0;
    //    sqrt_information_upper = pow(1.0/pix_noise, 2) * Matrix3d::Identity(); // one PLP (2d) and one epipolar (1d) factor
}

// Destructor
FactorTrifocal::~FactorTrifocal()
{
}

inline FeatureBasePtr FactorTrifocal::getFeaturePrev()
{
    return feature_prev_ptr_.lock();
}

template<typename T>
bool FactorTrifocal::operator ()( const T* const _pos1,
                                          const T* const _quat1,
                                          const T* const _pos2,
                                          const T* const _quat2,
                                          const T* const _pos3,
                                          const T* const _quat3,
                                          const T* const _sen_pos,
                                          const T* const _sen_quat,
                                          T* _residuals) const
{

    // MAPS
    Map<const Matrix<T,3,1> > wtr1(_pos1);
    Map<const Quaternion<T> > wqr1(_quat1);
    Map<const Matrix<T,3,1> > wtr2(_pos2);
    Map<const Quaternion<T> > wqr2(_quat2);
    Map<const Matrix<T,3,1> > wtr3(_pos3);
    Map<const Quaternion<T> > wqr3(_quat3);
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
inline void FactorTrifocal::expectation(const MatrixBase<D1>&     _wtr1,
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
        typedef Eigen::Transform<T, 3, Eigen::Isometry> TransformType;

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
         * There is a 3d point P, noted P1 when expressed in C1 and P2 when expressed in C2:
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
         *   P1' * c1Ec2 * P2    = 0 <--- Epipolar factor
         *
         * therefore:
         *   c1Ec2 = [T]x * R        <--- Essential matrix
         *
         * or, if we prefer the factor P2' * c2Ec1 * P1 = 0,
         *   c2Ec1 = c1Ec2' = R' * [T]x (we obviate the sign change)
         */
        Matrix<T,3,3> Rtr = c1Hc2.matrix().block(0,0,3,3).transpose();
        Matrix<T,3,1> t   = c1Hc2.matrix().block(0,3,3,1);
        _c2Ec1 = Rtr * skew(t);
//        _c2Ec1 =  c1Hc2.rotation().transpose() * skew(c1Hc2.translation()) ;
}

template<typename T, typename D1>
inline Matrix<T, 3, 1> FactorTrifocal::residual(const vision_utils::TrifocalTensorBase<T>& _tensor,
                                                        const MatrixBase<D1>& _c2Ec1) const
{
    // 1. COMMON COMPUTATIONS

    // m1, m2, m3: canonical pixels in cams 1,2,3 -- canonical means m = K.inv * u, with _u_ a homogeneous pixel [ux; uy; 1].
    Matrix<T,3,1> m1(pixel_canonical_1_.cast<T>());
    Matrix<T,3,1> m2(pixel_canonical_2_.cast<T>());
    Matrix<T,3,1> m3(pixel_canonical_3_.cast<T>());

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
inline Matrix<T, 3, 1> FactorTrifocal::error_jacobians(const vision_utils::TrifocalTensorBase<T>& _tensor,
                                                               const MatrixBase<D1>& _c2Ec1,
                                                               MatrixBase<D2>& _J_e_m1,
                                                               MatrixBase<D3>& _J_e_m2,
                                                               MatrixBase<D4>& _J_e_m3)
{
    // 1. COMMON COMPUTATIONS

    // m1, m2, m3: canonical pixels in cams 1,2,3 -- canonical means m = K.inv * u, with _u_ a homogeneous pixel [ux; uy; 1].
    Matrix<T,3,1> m1(pixel_canonical_1_.cast<T>());
    Matrix<T,3,1> m2(pixel_canonical_2_.cast<T>());
    Matrix<T,3,1> m3(pixel_canonical_3_.cast<T>());

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

// Print function
template<class T, int ROWS, int COLS>
void FactorTrifocal::print_matrix(const Eigen::Matrix<T, ROWS, COLS>& _mat) const
{}

template<int ROWS, int COLS>
void FactorTrifocal::print_matrix(const Eigen::Matrix<double, ROWS, COLS>& _mat) const
{
    std::cout << _mat << std::endl;
}

template<class T>
void FactorTrifocal::print_scalar(const T& _val) const
{}

void FactorTrifocal::print_scalar(const double& _val) const
{
    std::cout << _val << std::endl;
}

}    // namespace wolf

#endif /* _FACTOR_AUTODIFF_TRIFOCAL_H_ */
