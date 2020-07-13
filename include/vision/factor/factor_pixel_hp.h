#ifndef FACTOR_pixel_hp_H
#define FACTOR_pixel_hp_H

//Wolf includes
#include "vision/landmark/landmark_hp.h"
#include "vision/sensor/sensor_camera.h"
#include "vision/math/pinhole_tools.h"

#include "core/factor/factor_autodiff.h"

#include <iomanip> //setprecision

namespace wolf {

WOLF_PTR_TYPEDEFS(FactorPixelHp);
    
//class    
class FactorPixelHp : public FactorAutodiff<FactorPixelHp, 2, 3, 4, 3, 4, 4>
{
    protected:

        Eigen::Vector4d intrinsic_;
        Eigen::VectorXd distortion_;

    public:

        FactorPixelHp(const FeatureBasePtr&   _ftr_ptr,
                      const LandmarkHpPtr&   _landmark_ptr,
                      const ProcessorBasePtr& _processor_ptr,
                      bool              _apply_loss_function,
                      FactorStatus  _status = FAC_ACTIVE);

        ~FactorPixelHp() override = default;

        std::string getTopology() const override
        {
            return std::string("LMK");
        }

        template<typename T>
        void expectation(const T* const _frame_p,
                         const T* const _frame_o,
                         const T* const _sensor_p,
                         const T* const _sensor_o,
                         const T* const _lmk_hmg,
                         T* _expectation) const;

        Eigen::VectorXd expectation() const;

        template<typename T>
        bool operator ()(const T* const _frame_p,
                         const T* const _frame_o,
                         const T* const _sensor_p,
                         const T* const _sensor_o,
                         const T* const _lmk_hmg,
                         T* _residuals) const;
};

inline FactorPixelHp::FactorPixelHp(const FeatureBasePtr&   _ftr_ptr,
                                    const LandmarkHpPtr&   _landmark_ptr,
                                    const ProcessorBasePtr& _processor_ptr,
                                    bool             _apply_loss_function,
                                    FactorStatus _status) :
        FactorAutodiff<FactorPixelHp, 2, 3, 4, 3, 4, 4>("PIXELHP",
                                                        _ftr_ptr,
                                                        nullptr,
                                                        nullptr,
                                                        nullptr,
                                                        _landmark_ptr,
                                                        _processor_ptr,
                                                        _apply_loss_function,
                                                        _status,
                                                        _ftr_ptr->getCapture()->getFrame()->getP(),
                                                        _ftr_ptr->getCapture()->getFrame()->getO(),
                                                        _ftr_ptr->getCapture()->getSensorP(),
                                                        _ftr_ptr->getCapture()->getSensorO(),
                                                        _landmark_ptr->getP()),
        intrinsic_(_ftr_ptr->getCapture()->getSensor()->getIntrinsic()->getState())
{
//	std::cout << "FactorPixelHp::Constructor\n";
    // obtain some intrinsics from provided sensor
    distortion_ = (std::static_pointer_cast<SensorCamera>(_ftr_ptr->getCapture()->getSensor()))->getDistortionVector();
}

inline Eigen::VectorXd FactorPixelHp::expectation() const
{
    FrameBasePtr frm = getFeature()->getCapture()->getFrame();
    SensorBasePtr sen  = getFeature()->getCapture()->getSensor();
    LandmarkBasePtr lmk      = getLandmarkOther();

    const Eigen::MatrixXd frame_pos = frm->getP()->getState();
    const Eigen::MatrixXd frame_ori = frm->getO()->getState();
    const Eigen::MatrixXd sensor_pos  = sen ->getP()->getState();
    const Eigen::MatrixXd sensor_ori  = sen ->getO()->getState();
    const Eigen::MatrixXd lmk_pos_hmg       = lmk        ->getP()->getState();

    Eigen::Vector2d exp;
    expectation(frame_pos.data(), frame_ori.data(), sensor_pos.data(), sensor_ori.data(),
    			lmk_pos_hmg.data(), exp.data());

    return exp;
}

template<typename T>
inline void FactorPixelHp::expectation(const T* const _frame_p,
                                       const T* const _frame_o,
                                       const T* const _sensor_p,
                                       const T* const _sensor_o,
                                       const T* const _lmk_hmg,
                                       T* _expectation) const
{
    using namespace Eigen;

    // All involved transforms typedef
    typedef Eigen::Transform<T, 3, Eigen::Isometry> TransformType;

    // world to current robot transform
    Map<const Matrix<T, 3, 1> > p_w_r(_frame_p);
    Translation<T, 3>           t_w_r(p_w_r);
    Map<const Quaternion<T> >   q_w_r(_frame_o);
    TransformType               T_w_r = t_w_r * q_w_r;

    // current robot to current camera transform
    Map<const Matrix<T, 3, 1> > p_r_c(_sensor_p);
    Translation<T, 3>           t_r_c(p_r_c);
    Map<const Quaternion<T> >  	q_r_c(_sensor_o);
    TransformType       		T_r_c = t_r_c * q_r_c;

    // hmg point in current camera frame C
    Eigen::Map<const Eigen::Matrix<T, 4, 1> > landmark_hmg(_lmk_hmg);
    Eigen::Matrix<T, 4, 1> landmark_hmg_c = T_r_c .inverse(Eigen::Isometry)
                                           * T_w_r .inverse(Eigen::Isometry)
                                           * landmark_hmg;

    //std::cout << "p_w_r = \n\t" << _frame_p[0] << "\n\t" << _frame_p[1] << "\n\t" << _frame_p[2] << "\n";
//    std::cout << "q_w_r = \n\t" << _frame_o[0] << "\n\t" << _frame_o[1] << "\n\t" << _frame_o[2] << "\n\t" << _frame_o[3] << "\n";
//    std::cout << "p_r_c = \n\t" << _sensor_p[0] << "\n\t" << _sensor_p[1] << "\n\t" << _sensor_p[2] << "\n";
//    std::cout << "q_r_c = \n\t" << _sensor_o[0] << "\n\t" << _sensor_o[1] << "\n\t" << _sensor_o[2] << "\n\t" << _sensor_o[3] << "\n";
//    std::cout << "landmark_hmg_c = \n\t" << landmark_hmg_c(0) << "\n\t" << landmark_hmg_c(1) << "\n\t" << landmark_hmg_c(2) << "\n\t" << landmark_hmg_c(3) << "\n";

    // lmk direction vector
    Eigen::Matrix<T, 3, 1> v_dir = landmark_hmg_c.head(3);

    // lmk inverse distance
    T rho = landmark_hmg_c(3);

    // camera parameters
    Matrix<T, 4, 1> intrinsic = intrinsic_.cast<T>();
    Eigen::Matrix<T, Eigen::Dynamic, 1> distortion = distortion_.cast<T>();

    // project point and exit
    Eigen::Map<Eigen::Matrix<T, 2, 1> > expectation(_expectation);
    expectation = pinhole::projectPoint(intrinsic, distortion, v_dir/rho);

//    std::cout << "expectation = \n\t" << expectation(0) << "\n\t" << expectation(1) << "\n";

}

template<typename T>
inline bool FactorPixelHp::operator ()(const T* const _frame_p,
                                       const T* const _frame_o,
                                       const T* const _sensor_p,
                                       const T* const _sensor_o,
                                       const T* const _lmk_hmg,
                                       T* _residuals) const
{
    // expected
    Eigen::Matrix<T, 2, 1> expected;
    expectation(_frame_p, _frame_o, _sensor_p, _sensor_o, _lmk_hmg, expected.data());

    // measured
    Eigen::Matrix<T, 2, 1> measured = getMeasurement().cast<T>();

    // residual
    Eigen::Map<Eigen::Matrix<T, 2, 1> > residuals(_residuals);
    residuals = getMeasurementSquareRootInformationUpper().cast<T>() * (expected - measured);
    return true;
}

} // namespace wolf

#endif // FACTOR_AHP_H
