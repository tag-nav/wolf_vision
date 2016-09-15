#ifndef CONSTRAINT_IMAGE_H
#define CONSTRAINT_IMAGE_H

//Wolf includes
#include "constraint_sparse.h"
#include "landmark_AHP.h"
#include "sensor_camera.h"
#include "pinholeTools.h"
#include "feature_point_image.h"

namespace wolf {

class ConstraintImage : public ConstraintSparse<2, 3, 4, 3, 4, 4>
{
    protected:
        Eigen::Vector4s anchor_sensor_intrinsics_;
        Eigen::Vector3s anchor_sensor_extrinsics_p_;
        Eigen::Vector4s anchor_sensor_extrinsics_o_;
        FeaturePointImage feature_image_;

    public:
        static const unsigned int N_BLOCKS = 5; //TODO: Prueba a comentarlo

        ConstraintImage(FeatureBase* _ftr_ptr, FrameBase* _current_frame_ptr, LandmarkAHP* _landmark_ptr,
                        bool _apply_loss_function = false, ConstraintStatus _status = CTR_ACTIVE) :
                ConstraintSparse<2, 3, 4, 3, 4, 4>(CTR_AHP, _landmark_ptr, _apply_loss_function, _status,
                                             _current_frame_ptr->getPPtr(), _current_frame_ptr->getOPtr(), _landmark_ptr->getAnchorFrame()->getPPtr()
                                                   ,_landmark_ptr->getAnchorFrame()->getOPtr(),_landmark_ptr->getPPtr()),
                anchor_sensor_intrinsics_(_ftr_ptr->getCapturePtr()->getSensorPtr()->getIntrinsicPtr()->getVector()),
                anchor_sensor_extrinsics_p_(_ftr_ptr->getCapturePtr()->getSensorPPtr()->getVector()),
                anchor_sensor_extrinsics_o_(_ftr_ptr->getCapturePtr()->getSensorOPtr()->getVector()),
                feature_image_(*((FeaturePointImage*)_ftr_ptr))
        {
            setType("AHP");

        }

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~ConstraintImage()
        {
            //
        }

        template<typename T>
        bool operator ()(const T* const _current_frame_p, const T* const _current_frame_o, const T* const _anchor_frame_p,
                         const T* const _anchor_frame_o, const T* const _lmk_hmg, T* _residuals) const
        {

            std::cout << "\n============== constraint new method ============" << std::endl;

            Eigen::Matrix<T, 4, 1> k_params = anchor_sensor_intrinsics_.cast<T>();
            Eigen::Matrix<T, 3, 3> K;
            K(0, 0) = k_params(2);
            K(0, 1) = T(0);
            K(0, 2) = k_params(0);
            K(1, 0) = T(0);
            K(1, 1) = k_params(3);
            K(1, 2) = k_params(1);
            K(2, 0) = T(0);
            K(2, 1) = T(0);
            K(2, 2) = T(1);



            Eigen::Map<const Eigen::Matrix<T, 3, 1> > pwr1(_current_frame_p);
            Eigen::Map<const Eigen::Matrix<T, 3, 1> > pwr0(_anchor_frame_p);
            Eigen::Matrix<T, 3, 1> prc = anchor_sensor_extrinsics_p_.cast<T>();

            Eigen::Quaternion<T> qwr1, qwr0, qrc;
            qwr0 = _anchor_frame_o;
            qwr1 = _current_frame_o;
            qrc = anchor_sensor_extrinsics_o_.cast<T>();

//            Eigen::Matrix<T,3,1>  phc, phw ;
            Eigen::Map<const Eigen::Matrix<T, 4, 1> > landmark_map(_lmk_hmg);


            Eigen::Matrix<T,4,4> M_R0_C0, M_W_R0, M_R1_W, M_C1_R1;

//            std::cout << "rotation matrix:\n"
//                      << qwr1.matrix()(0,0) << "\t" << qwr1.matrix()(0,1) << "\t" << qwr1.matrix()(0,2) << std::endl
//                      << qwr1.matrix()(1,0) << "\t" << qwr1.matrix()(1,1) << "\t" << qwr1.matrix()(1,2) << std::endl
//                      << qwr1.matrix()(2,0) << "\t" << qwr1.matrix()(2,1) << "\t" << qwr1.matrix()(2,2) << std::endl;


            // FROM CAMERA0 TO WORLD

            M_R0_C0.block(0,0,3,3) << qrc.matrix();
            M_R0_C0.col(3).head(3) << prc;
            M_R0_C0.row(3) << (T)0, (T)0, (T)0, (T)1;

            M_W_R0.block(0,0,3,3) << qwr0.matrix();
            M_W_R0.col(3).head(3) << pwr0;
            M_W_R0.row(3) << (T)0, (T)0, (T)0, (T)1;

            Eigen::Matrix<T,4,1> test;
            test = M_W_R0*M_R0_C0*landmark_map;

//            std::cout << "\ntest:\n" << test(0) << "\t" << test(1) << "\t" << test(2) << "\t" << test(3) << std::endl;


            // FROM WORLD TO CAMERA1

            M_R1_W.block(0,0,3,3) << qwr1.matrix().transpose();
            M_R1_W.col(3).head(3) << (-qwr1.matrix().transpose()*pwr1);
            M_R1_W.row(3) << (T)0, (T)0, (T)0, (T)1;

            M_C1_R1.block(0,0,3,3) << qrc.matrix().transpose();
            M_C1_R1.col(3).head(3) << (-qrc.matrix().transpose()*prc);
            M_C1_R1.row(3) << (T)0, (T)0, (T)0, (T)1;

            Eigen::Matrix<T,4,1> test2;
            test2 = M_C1_R1*M_R1_W*test;

//            std::cout << "\ntest2:\n" << test2(0) << "\t" << test2(1) << "\t" << test2(2) << "\t" << test2(3) << std::endl;

            Eigen::Matrix<T,3,1> v;
            v(0) = test2(0);
            v(1) = test2(1);
            v(2) = test2(2);
//            T inverse_dist = test2(3); // inverse distance

            // ==================================================
            Eigen::Matrix<T, 3, 1> u_;
            u_ = K * v;

            Eigen::Matrix<T, 2, 1> u_12;
            u_12(0) = u_(0);
            u_12(1) = u_(1);

            Eigen::Matrix<T, 2, 1> u;
            if (u_(2) != T(0))
            {
                u = u_12 / u_(2);
            }
            else
            {
                u = u_12;
            }
//            std::cout << "\nu:\n" << u(0) << "\t" << u(1) << std::endl;

            // ==================================================
            //    std::cout << "==============================================\nCONSTRAINT INFO" << std::endl;
            //    std::cout << "Estimation of the Projection:\n\t" << u(0) << "\n\t" << u(1) << std::endl;
//            std::cout << "Feature measurement:\n" << getMeasurement() << std::endl;
            Eigen::Matrix<T, 2, 1> feature_pos = getMeasurement().cast<T>();

            Eigen::Map<Eigen::Matrix<T, 2, 1> > residualsmap(_residuals);
            residualsmap = getMeasurementSquareRootInformation().cast<T>() * (u - feature_pos);
            std::cout << "\nRESIDUALS:\n" << residualsmap[0] << "\t" << residualsmap[1] << std::endl;





            /* TRANSFORM MATRIX APPROACH */


//            Eigen::Transform<T,4,4> test_mat;
//            test_mat(0,0) = qrc.matrix()(0,0); test_mat(0,1) = qrc.matrix()(0,1); test_mat(0,2) = qrc.matrix()(0,2); test_mat(0,3) = prc(0);
//            test_mat(1,0) = qrc.matrix()(1,0); test_mat(1,1) = qrc.matrix()(1,1); test_mat(1,2) = qrc.matrix()(1,2); test_mat(1,3) = prc(1);
//            test_mat(2,0) = qrc.matrix()(2,0); test_mat(2,1) = qrc.matrix()(2,1); test_mat(2,2) = qrc.matrix()(2,2); test_mat(2,3) = prc(2);
//            test_mat(3,0) = (T)0; test_mat(3,1) = (T)0; test_mat(3,2) = (T)0; test_mat(3,3) = (T)1;

//            std::cout << "Transform matrix:\n"
//                      << test_mat(0,0) << "\t" << test_mat(0,1) << "\t" << test_mat(0,2) << "\t" << test_mat(0,3) << std::endl
//                      << test_mat(1,0) << "\t" << test_mat(1,1) << "\t" << test_mat(1,2) << "\t" << test_mat(1,3) << std::endl
//                      << test_mat(2,0) << "\t" << test_mat(2,1) << "\t" << test_mat(2,2) << "\t" << test_mat(2,3) << std::endl
//                      << test_mat(3,0) << "\t" << test_mat(3,1) << "\t" << test_mat(3,2) << "\t" << test_mat(3,3) << std::endl;


//            Eigen::Transform<T,4,4> t;
//            t(0,0) = (T)1; t(0,1) = (T)0; t(0,2) = (T)0; t(0,3) = (T)0;
//            t(1,0) = (T)0; t(1,1) = (T)1; t(1,2) = (T)0; t(1,3) = (T)3;
//            t(2,0) = (T)0; t(2,1) = (T)0; t(2,2) = (T)1; t(2,3) = (T)2;
//            t(3,0) = (T)0; t(3,1) = (T)0; t(3,2) = (T)0; t(3,3) = (T)1;

//            //Eigen::Translation<T,3> translation(prc);
//            Eigen::Translation<T,3> translation;
//            translation.x() = (T)3;
//            translation.y() = (T)2;
//            translation.z() = (T)0;

//            Eigen::Vector4s test_quaternion = {0,2,0,1};
//            Eigen::Quaternion<T> rotation;
//            rotation= test_quaternion.cast<T>();

//            std::cout << "rotation matrix:\n"
//                      << rotation.matrix()(0,0) << "\t" << rotation.matrix()(0,1) << "\t" << rotation.matrix()(0,2) << std::endl
//                      << rotation.matrix()(1,0) << "\t" << rotation.matrix()(1,1) << "\t" << rotation.matrix()(1,2) << std::endl
//                      << rotation.matrix()(2,0) << "\t" << rotation.matrix()(2,1) << "\t" << rotation.matrix()(2,2) << std::endl;

//            std::cout << "rotation transposed matrix:\n"
//                      << rotation.matrix().transpose()(0,0) << "\t" << rotation.matrix().transpose()(0,1) << "\t" << rotation.matrix().transpose()(0,2) << std::endl
//                      << rotation.matrix().transpose()(1,0) << "\t" << rotation.matrix().transpose()(1,1) << "\t" << rotation.matrix().transpose()(1,2) << std::endl
//                      << rotation.matrix().transpose()(2,0) << "\t" << rotation.matrix().transpose()(2,1) << "\t" << rotation.matrix().transpose()(2,2) << std::endl;

//            Eigen::Transform<T,3,Eigen::Affine> combined = translation * rotation;


//            std::cout << "Combined Transform matrix:\n"
//                      << combined(0,0) << "\t" << combined(0,1) << "\t" << combined(0,2) << "\t" << combined(0,3) << std::endl
//                      << combined(1,0) << "\t" << combined(1,1) << "\t" << combined(1,2) << "\t" << combined(1,3) << std::endl
//                      << combined(2,0) << "\t" << combined(2,1) << "\t" << combined(2,2) << "\t" << combined(2,3) << std::endl
//                      << combined(3,0) << "\t" << combined(3,1) << "\t" << combined(3,2) << "\t" << combined(3,3) << std::endl;

//            Eigen::Transform<T,3,Eigen::Affine> combined_inv = combined.inverse(Eigen::Affine);

//            std::cout << "Combined inverse Transform matrix:\n"
//                      << combined_inv(0,0) << "\t" << combined_inv(0,1) << "\t" << combined_inv(0,2) << "\t" << combined_inv(0,3) << std::endl
//                      << combined_inv(1,0) << "\t" << combined_inv(1,1) << "\t" << combined_inv(1,2) << "\t" << combined_inv(1,3) << std::endl
//                      << combined_inv(2,0) << "\t" << combined_inv(2,1) << "\t" << combined_inv(2,2) << "\t" << combined_inv(2,3) << std::endl
//                      << combined_inv(3,0) << "\t" << combined_inv(3,1) << "\t" << combined_inv(3,2) << "\t" << combined_inv(3,3) << std::endl;


//            /* NO SON MATRICES DE TRANSFORMACION, SOLO MATRICES NORMALES. BUSCA LA CLASE Eigen::Transform */

////            M_R_C.block(0,0,3,3) << qrc0.matrix();
////            M_R_C.col(3).head(3) << prc;
////            M_R_C.row(3) << (T)0, (T)0, (T)0, (T)1;


//            //phw = M_W_R*M_R_C*phc;



            /* END OF THE TRANSFORM MATRIX APPROACH */




            return true;
        }

        /** \brief Returns the jacobians computation method
         **/
        virtual JacobianMethod getJacobianMethod() const
        {
            return JAC_AUTO;
        }

//    public:
//        static wolf::ConstraintBase* create(FeatureBase* _feature_ptr, //
//                                            NodeBase* _correspondant_ptr)
//        {
//            return new ConstraintImage(_feature_ptr, (FrameBase*)_correspondant_ptr);
//        }

};

} // namespace wolf

#endif // CONSTRAINT_IMAGE_H
