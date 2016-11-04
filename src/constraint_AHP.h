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

        ConstraintAHP(FeatureBasePtr _ftr_ptr, FrameBasePtr _current_frame_ptr, LandmarkAHP::Ptr _landmark_ptr,
                        bool _apply_loss_function = false, ConstraintStatus _status = CTR_ACTIVE) :
                ConstraintSparse<2, 3, 4, 3, 4, 4>(CTR_AHP, _landmark_ptr, _apply_loss_function, _status,
                                             _current_frame_ptr->getPPtr(), _current_frame_ptr->getOPtr(), _landmark_ptr->getAnchorFrame()->getPPtr()
                                                   ,_landmark_ptr->getAnchorFrame()->getOPtr(),_landmark_ptr->getPPtr()),
                anchor_sensor_extrinsics_p_(_ftr_ptr->getCapturePtr()->getSensorPPtr()->getVector()),
                anchor_sensor_extrinsics_o_(_ftr_ptr->getCapturePtr()->getSensorOPtr()->getVector()),
                feature_image_(*std::static_pointer_cast<FeaturePointImage>(_ftr_ptr))
        {
            setType("AHP");

            frame_other_ptr_ = _landmark_ptr->getAnchorFrame();

            K_ = (std::static_pointer_cast<SensorCamera>(_ftr_ptr->getCapturePtr()->getSensorPtr()))->getIntrinsicMatrix();
            distortion_ = (std::static_pointer_cast<SensorCamera>(_ftr_ptr->getCapturePtr()->getSensorPtr()))->getDistortionVector();
        }

        virtual ~ConstraintAHP()
        {
            //
        }

        template<typename T>
        void expectation(const T* const _current_frame_p, const T* const _current_frame_o, const T* const _anchor_frame_p,
                                    const T* const _anchor_frame_o, const T* const _lmk_hmg, T* _expectation) const
        {
            // Maps over the input pointers
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

            // hmg point in C1 frame (current frame)
            Eigen::Matrix<T,4,1> landmark_hmg_c1;
            landmark_hmg_c1 = T_R1_C1.inverse(Eigen::Affine) * T_W_R1.inverse(Eigen::Affine) * T_W_R0 * T_R0_C0 * landmark_hmg_c0;
            //            std::cout << "\nlandmark_hmg_c1:\n" << landmark_hmg_c1(0) << "\t" << landmark_hmg_c1(1) << "\t" << landmark_hmg_c1(2) << "\t" << landmark_hmg_c1(3) << std::endl;


            // lmk direction vector
            Eigen::Matrix<T,3,1> v_dir = landmark_hmg_c1.head(3);

//            std::cout << "\nv_normalized:\n" << v_dir(0) << "\t" << v_dir(1) << "\t"
//                      << v_dir(2) << "\t" << landmark_hmg_c1(3) << std::endl;

            // projected point in canonical sensor
            Eigen::Matrix<T,2,1> point_undistorted;
            point_undistorted = v_dir.head(2)/v_dir(2);
            //            std::cout << "\nv:\n" << point_undistorted(0) << "\t" << point_undistorted(1) << std::endl;

            Eigen::Matrix<T,Eigen::Dynamic,1> distortion_vector = distortion_.cast<T>();
            //            std::cout << "\ndistortion_vector:\n" << distortion_vector(0) << "\t" << distortion_vector(1) << std::endl;

            // distort point
            Eigen::Matrix<T,2,1> point_distorted = pinhole::distortPoint(distortion_vector, point_undistorted);

            // pixellize
            Eigen::Map<Eigen::Matrix<T, 2, 1> > expectation(_expectation);
            expectation(0) = K(0,0) * point_distorted(0) + K(0,2);
            expectation(1) = K(1,1) * point_distorted(1) + K(1,2);

            //            std::cout << "constraint n[" << id() << "] _expectation: " << _expectation(0) << "\t" << _expectation(1) << std::endl;

        }

        Eigen::VectorXs expectation() const
        {
            Eigen::VectorXs exp(2);
            FrameBasePtr frm_current    = getFeaturePtr()->getCapturePtr()->getFramePtr();
            FrameBasePtr frm_anchor     = getFrameOtherPtr();
            LandmarkBasePtr lmk         = getLandmarkOtherPtr();
            const Scalar * const frame_current_pos  = frm_current->getPPtr()->getVector().data();
            const Scalar * const frame_current_ori  = frm_current->getOPtr()->getVector().data();
            const Scalar * const frame_anchor_pos   = frm_anchor->getPPtr()->getVector().data();
            const Scalar * const frame_anchor_ori   = frm_anchor->getOPtr()->getVector().data();
            const Scalar * const lmk_pos_hmg        = lmk->getPPtr()->getVector().data();
            expectation(frame_current_pos,
                        frame_current_ori,
                        frame_anchor_pos,
                        frame_anchor_ori,
                        lmk_pos_hmg,
                        exp.data());
            return exp;
        }

        template<typename T>
        bool operator ()(const T* const _current_frame_p, const T* const _current_frame_o, const T* const _anchor_frame_p,
                         const T* const _anchor_frame_o, const T* const _lmk_hmg, T* _residuals) const
        {
            std::cout << "operator: " << id() << std::endl;
            Eigen::Matrix<T, Eigen::Dynamic, 1> expected(2) ;
            expectation(_current_frame_p, _current_frame_o,  _anchor_frame_p,
                          _anchor_frame_o,  _lmk_hmg, expected.data()) ;

            Eigen::Matrix<T, 2, 1> measured = getMeasurement().cast<T>();

            Eigen::Map<Eigen::Matrix<T, 2, 1> > residuals(_residuals);

            residuals = getMeasurementSquareRootInformation().cast<T>() * (expected - measured);

            // debug info:
            Eigen::Map<const Eigen::Matrix< T, 4, 1> > landmark(_lmk_hmg);
//            std::cout << "\nLANDMARK  L" <<  getLandmarkOtherPtr()->id() << ":\n\t" << landmark[0] << "\n\t" << landmark[1] << "\n\t" << landmark[2] << "\n\t" << landmark[3] << std::endl;
//            std::cout << "\nRESIDUALS c" <<  id() << ":\n\t" << residuals[0] << "\n\t" << residuals[1] << std::endl;

            return true;
        }

        /** \brief Returns the jacobians computation method
         **/
        virtual JacobianMethod getJacobianMethod() const
        {
            return JAC_AUTO;
        }

        static ConstraintAHP::Ptr create(FeatureBasePtr _ftr_ptr, FrameBasePtr _frm_current_ptr, LandmarkAHP::Ptr _lmk_ahp_ptr)
        {
            // construct constraint
            Ptr ctr_ahp = std::make_shared<ConstraintAHP>(_ftr_ptr, _frm_current_ptr, _lmk_ahp_ptr);

            // link it to wolf tree
//            _ftr_ptr->addConstraint(ctr_ahp);
            ctr_ahp->setFrameOtherPtr(_lmk_ahp_ptr->getAnchorFrame());
            _lmk_ahp_ptr->getAnchorFrame()->addConstrainedBy(ctr_ahp);
            _lmk_ahp_ptr->addConstrainedBy(ctr_ahp);
            return  ctr_ahp;
        }


};

} // namespace wolf


#endif // CONSTRAINT_AHP_H
