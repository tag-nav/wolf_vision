#ifndef CONSTRAINT_AHP_H
#define CONSTRAINT_AHP_H

//Wolf includes
#include "constraint_sparse.h"
#include "landmark_AHP.h"
#include "sensor_camera.h"
#include "pinholeTools.h"
#include "feature_point_image.h"

#include <iomanip> //setprecision

namespace wolf {

class ConstraintAHP : public ConstraintSparse<2, 3, 4, 3, 4, 4>
{
    public:
        typedef std::shared_ptr<ConstraintAHP> Ptr;
        typedef std::weak_ptr<ConstraintAHP> WPtr;

    protected:
        Eigen::Vector3s anchor_sensor_extrinsics_p_;
        Eigen::Vector4s anchor_sensor_extrinsics_o_;
        Eigen::Matrix3s K_;
        Eigen::VectorXs distortion_;
        FeaturePointImage feature_image_;


    public:

        ConstraintAHP(FeatureBasePtr _ftr_ptr, FrameBasePtr _current_frame_ptr, std::shared_ptr<LandmarkAHP> _landmark_ptr,
                        bool _apply_loss_function = false, ConstraintStatus _status = CTR_ACTIVE) :
                ConstraintSparse<2, 3, 4, 3, 4, 4>(CTR_AHP, _landmark_ptr, _apply_loss_function, _status,
                                             _current_frame_ptr->getPPtr(), _current_frame_ptr->getOPtr(), _landmark_ptr->getAnchorFrame()->getPPtr()
                                                   ,_landmark_ptr->getAnchorFrame()->getOPtr(),_landmark_ptr->getPPtr()),
                anchor_sensor_extrinsics_p_(_ftr_ptr->getCapturePtr()->getSensorPPtr()->getVector()),
                anchor_sensor_extrinsics_o_(_ftr_ptr->getCapturePtr()->getSensorOPtr()->getVector()),
                feature_image_(*std::static_pointer_cast<FeaturePointImage>(_ftr_ptr))
        {
            setType("AHP");

            K_ = (std::static_pointer_cast<SensorCamera>(_ftr_ptr->getCapturePtr()->getSensorPtr()))->getIntrinsicMatrix();
            distortion_ = (std::static_pointer_cast<SensorCamera>(_ftr_ptr->getCapturePtr()->getSensorPtr()))->getDistortionVector();
        }

        virtual ~ConstraintAHP()
        {
            //
        }

        template<typename T>
        Eigen::VectorXs convert(const Eigen::Matrix<T,2,1>& v)
        {
            return Eigen::VectorXs(0);
        }

        template<typename T>
        void expectation(const T* const _current_frame_p, const T* const _current_frame_o, const T* const _anchor_frame_p,
                                    const T* const _anchor_frame_o, const T* const _lmk_hmg, T* _expectation) const
        {
            Eigen::Matrix<T, 3, 3> K = K_.cast<T>();
            Eigen::Map<const Eigen::Matrix<T, 4, 1> > landmark_hmg_c0(_lmk_hmg);

            Eigen::Map<const Eigen::Matrix<T, 3, 1> > pwr1(_current_frame_p);
            Eigen::Map<const Eigen::Matrix<T, 3, 1> > pwr0(_anchor_frame_p);
            Eigen::Matrix<T, 3, 1> prc = anchor_sensor_extrinsics_p_.cast<T>();

            Eigen::Quaternion<T> qwr1, qwr0, qrc;
            qwr0 = _anchor_frame_o;
            qwr1 = _current_frame_o;
            qrc = anchor_sensor_extrinsics_o_.cast<T>();


            /* TRANSFORM MATRIX APPROACH */

            Eigen::Translation<T,3> twr1, twr0, trc;
            twr1.x() = pwr1(0); twr1.y() = pwr1(1); twr1.z() = pwr1(2);
            twr0.x() = pwr0(0); twr0.y() = pwr0(1); twr0.z() = pwr0(2);
            trc.x() = prc(0); trc.y() = prc(1); trc.z() = prc(2);


            Eigen::Transform<T,3,Eigen::Affine> T_W_R0, T_W_R1, T_R0_C0, T_R1_C1;

            T_W_R0 = twr0 * qwr0;
            T_W_R1 = twr1 * qwr1;
            T_R0_C0 = trc * qrc;
            T_R1_C1 = T_R0_C0;

            Eigen::Matrix<T,4,1> landmark_hmg_c1;
            landmark_hmg_c1 = T_R1_C1.inverse(Eigen::Affine) * T_W_R1.inverse(Eigen::Affine) * T_W_R0 * T_R0_C0 * landmark_hmg_c0;
            std::cout << "\nlandmark_hmg_c1:\n" << landmark_hmg_c1(0) << "\t" << landmark_hmg_c1(1) << "\t" << landmark_hmg_c1(2) << "\t" << landmark_hmg_c1(3) << std::endl;


            Eigen::Matrix<T,3,1> v_normalized;
            v_normalized = landmark_hmg_c1.head(3);// /landmark_hmg_c1(3);
//            T inverse_dist_c1 = landmark_hmg_c1(3); // inverse distance

            std::cout << "\nv_normalized:\n" << v_normalized(0) << "\t" << v_normalized(1) << "\t"
                      << v_normalized(2) << "\t" << landmark_hmg_c1(3) << std::endl;

            Eigen::Matrix<T,2,1> v;
            v = v_normalized.head(2)/v_normalized(2);
            std::cout << "\nv:\n" << v(0) << "\t" << v(1) << std::endl;

            Eigen::Matrix<T,2,1> distored_point;
            Eigen::Matrix<T,Eigen::Dynamic,1> distortion_vector = distortion_.cast<T>();
            std::cout << "\ndistortion_vector:\n" << distortion_vector(0) << "\t" << distortion_vector(1) << std::endl;

            Eigen::Matrix<T,2,1> distored_point_pinhole;
            distored_point_pinhole = pinhole::distortPoint(distortion_vector, v);

            T r2 = v.squaredNorm(); // this is the norm squared: r2 = ||u||^2

            T s = (T)1.0;
            T r2i = (T)1.0;
            for (int i = 0; i < distortion_vector.cols() ; i++) { //   here we are doing:
                r2i = r2i * r2;                                   //   r2i = r^(2*(i+1))
                s = s + (distortion_vector(i) * r2i);             //   s = 1 + d_0 * r^2 + d_1 * r^4 + d_2 * r^6 + ...
            }
            if (s < (T)0.6) s = (T)1.0;
            distored_point(0) = s * v(0);
            distored_point(1) = s * v(1);
            std::cout << "\ndistored_point:\n" << distored_point(0) << "\t" << distored_point(1) << std::endl;
            std::cout << "\ndistored_point_pinhole:\n" << distored_point_pinhole(0) << "\t" << distored_point_pinhole(1) << std::endl;

//            std::cout << "K: " << K << std::endl;

            Eigen::Map<Eigen::Matrix<T, 2, 1> > expectation(_expectation);
            expectation(0) = K(0,0)*distored_point_pinhole(0)+K(0,2);
            expectation(1) = K(1,1)*distored_point_pinhole(1)+K(1,2);

//            std::cout << "constraint n[" << id() << "] _expectation: " << _expectation(0) << "\t" << _expectation(1) << std::endl;

//            return u;
        }

        template<typename T>
        bool operator ()(const T* const _current_frame_p, const T* const _current_frame_o, const T* const _anchor_frame_p,
                         const T* const _anchor_frame_o, const T* const _lmk_hmg, T* _residuals) const
        {

            Eigen::Matrix<T, Eigen::Dynamic, 1> u ;
            expectation(_current_frame_p, _current_frame_o,  _anchor_frame_p,
                          _anchor_frame_o,  _lmk_hmg, u.data()) ;
            // ==================================================

            //    std::cout << "\nCONSTRAINT INFO" << std::endl;

            Eigen::Matrix<T, 2, 1> feature_pos = getMeasurement().cast<T>();
            std::cout << __FILE__ <<":"<< __FUNCTION__ <<"():"<< __LINE__ << std::endl;

            Eigen::Map<Eigen::Matrix<T, 2, 1> > residualsmap(_residuals);
            std::cout << __FILE__ <<":"<< __FUNCTION__ <<"():"<< __LINE__ << std::endl;
//            std::cout << "SquareRootInformation: " << getMeasurementSquareRootInformation() << std::endl;
            residualsmap = getMeasurementSquareRootInformation().cast<T>() * (u - feature_pos);
//            std::cout << "\nRESIDUALS:\n" << residualsmap[0] << "\t" << residualsmap[1] << std::endl;

            std::cout << __FILE__ <<":"<< __FUNCTION__ <<"():"<< __LINE__ << std::endl;

            return true;
        }

        /** \brief Returns the jacobians computation method
         **/
        virtual JacobianMethod getJacobianMethod() const
        {
            return JAC_AUTO;
        }


};

} // namespace wolf


#endif // CONSTRAINT_AHP_H
