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
        Eigen::Vector4s intrinsics_;
        Eigen::Vector3s extrinsics_p_;
        Eigen::Vector4s extrinsics_o_;
        Eigen::Vector3s anchor_p_;
        Eigen::Vector4s anchor_o_;
        FeaturePointImage feature_image_;

    public:
        static const unsigned int N_BLOCKS = 5; //TODO: Prueba a comentarlo

        ConstraintImage(FeatureBase* _ftr_ptr, FrameBase* _frame_ptr, LandmarkAHP* _landmark_ptr,
                        bool _apply_loss_function = false, ConstraintStatus _status = CTR_ACTIVE) :
                ConstraintSparse<2, 3, 4, 3, 4, 4>(_ftr_ptr, CTR_EPIPOLAR, _landmark_ptr, _apply_loss_function, _status,
                                             _frame_ptr->getPPtr(), _frame_ptr->getOPtr(), _landmark_ptr->getAnchorFrame()->getPPtr()
                                                   ,_landmark_ptr->getAnchorFrame()->getOPtr(),_landmark_ptr->getPPtr()),
                intrinsics_(_ftr_ptr->getCapturePtr()->getSensorPtr()->getIntrinsicPtr()->getVector()),
                extrinsics_p_(_ftr_ptr->getCapturePtr()->getSensorPPtr()->getVector()),
                extrinsics_o_(_ftr_ptr->getCapturePtr()->getSensorOPtr()->getVector()),
                anchor_p_(_landmark_ptr->getAnchorFrame()->getPPtr()->getVector()),
                anchor_o_(_landmark_ptr->getAnchorFrame()->getOPtr()->getVector()),
                feature_image_(*((FeaturePointImage*)_ftr_ptr))
        {
            setType("IMAGE");

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
        bool operator ()(const T* const _probot, const T* const _orobot, const T* const _lmk, T* _residuals) const;

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
template<typename T>
inline bool ConstraintImage::operator ()(const T* const _probot, const T* const _orobot, const T* const _lmk,
                                         T* _residuals) const
{
    // Do the magic here
    Eigen::Map<const Eigen::Matrix<T, 3, 1> > translation_F1_world2robot(_probot); //translation_world2robot
    Eigen::Map<const Eigen::Matrix<T, 4, 1> > orobotmap(_orobot);
    Eigen::Map<Eigen::Matrix<T, 2, 1> > residualsmap(_residuals);

    std::cout << "probot:\n" << translation_F1_world2robot(0) << "\t" << translation_F1_world2robot(1)
              << "\t" << translation_F1_world2robot(2) << std::endl;
    std::cout << "orobot:\n" << orobotmap(0) << "\t" << orobotmap(1) << "\t" << orobotmap(2) << "\t" << orobotmap(3)<< std::endl;
    std::cout << "residuals:\n" << residualsmap(0) << "\t" << residualsmap(1) << std::endl;

    Eigen::Matrix<T,4,1> k_params = intrinsics_.cast<T>();
    Eigen::Matrix<T,3,3> K;
    K(0,0) = k_params(0);
    K(0,1) = 0;
    K(0,2) = k_params(2);
    K(1,0) = 0;
    K(1,1) = k_params(1);
    K(1,2) = k_params(3);
    K(2,0) = 0;
    K(2,1) = 0;
    K(2,2) = 1;

    std::cout << "K matrix:\n" << K(0,0) << "\t" << K(0,1) << "\t" << K(0,2) << "\n"
              << K(1,0) << "\t" << K(1,1) << "\t" << K(1,2) << "\n"
              << K(2,0) << "\t" << K(2,1) << "\t" << K(2,2) << "\n" << std::endl;

    Eigen::Matrix<T,3,1> translation_robot2camera = extrinsics_p_.cast<T>();
    Eigen::Matrix<T,4,1> sensor_orientation = extrinsics_o_.cast<T>();
    std::cout << "translation robot to camera:\n" << translation_robot2camera(0) << "\t" << translation_robot2camera(1)
              << "\t" << translation_robot2camera(2) << std::endl;
    std::cout << "orientation robot to camera:\n" << sensor_orientation(0) << "\t" << sensor_orientation(1) << "\t"
              << sensor_orientation(2) << "\t" << sensor_orientation(3)<< std::endl;

    Eigen::Matrix<T,3,1> translation_F0_world2robot = anchor_p_.cast<T>();
    Eigen::Matrix<T,4,1> anchor_orientation = anchor_o_.cast<T>();
    std::cout << "translation F0 world to robot:\n" << translation_F0_world2robot(0) << "\t" << translation_F0_world2robot(1)
              << "\t" << translation_F0_world2robot(2) << std::endl;
    std::cout << "orientation F0 world to robot:\n" << anchor_orientation(0) << "\t" << anchor_orientation(1) << "\t"
              << anchor_orientation(2) << "\t" << anchor_orientation(3)<< std::endl;


    Eigen::Map<const Eigen::Matrix<T, 4, 1> > landmarkmap(_lmk);
    //Eigen::Matrix<T,1,1> inverse_depth;
    Eigen::Matrix<T,3,1> m;
    m(0) = landmarkmap(0);
    m(1) = landmarkmap(1);
    m(2) = landmarkmap(2);
    std::cout << "m:\n" << m(0) << "\t" << m(1) << "\t" << m(2) << "\ninverse depth: " << landmarkmap(3) << std::endl;

    /* making the rotations manually now */
    Eigen::Matrix<T,3,3> rotation_F1_world2robot;
    rotation_F1_world2robot(0,0) = pow(orobotmap(3),2) + pow(orobotmap(0),2) - pow(orobotmap(1),2) - pow(orobotmap(2),2);
    rotation_F1_world2robot(0,1) = 2*(orobotmap(0)*orobotmap(1) + orobotmap(3)*orobotmap(2));
    rotation_F1_world2robot(0,2) = 2*(orobotmap(0)*orobotmap(2) - orobotmap(3)*orobotmap(1));;
    rotation_F1_world2robot(1,0) = 2*(orobotmap(0)*orobotmap(1) - orobotmap(3)*orobotmap(2));;
    rotation_F1_world2robot(1,1) = pow(orobotmap(3),2) - pow(orobotmap(0),2) + pow(orobotmap(1),2) - pow(orobotmap(2),2);
    rotation_F1_world2robot(1,2) = 2*(orobotmap(1)*orobotmap(2) + orobotmap(3)*orobotmap(0));
    rotation_F1_world2robot(2,0) = 2*(orobotmap(0)*orobotmap(2) + orobotmap(3)*orobotmap(1));
    rotation_F1_world2robot(2,1) = 2*(orobotmap(1)*orobotmap(2) - orobotmap(3)*orobotmap(0));
    rotation_F1_world2robot(2,2) = pow(orobotmap(3),2) - pow(orobotmap(0),2) - pow(orobotmap(1),2) + pow(orobotmap(2),2);

    Eigen::Matrix<T,3,3> rotation_F0_world2robot;
    rotation_F0_world2robot(0,0) = pow(anchor_orientation(3),2) + pow(anchor_orientation(0),2) - pow(anchor_orientation(1),2) - pow(anchor_orientation(2),2);
    rotation_F0_world2robot(0,1) = 2*(anchor_orientation(0)*anchor_orientation(1) + anchor_orientation(3)*anchor_orientation(2));
    rotation_F0_world2robot(0,2) = 2*(anchor_orientation(0)*anchor_orientation(2) - anchor_orientation(3)*anchor_orientation(1));;
    rotation_F0_world2robot(1,0) = 2*(anchor_orientation(0)*anchor_orientation(1) - anchor_orientation(3)*anchor_orientation(2));;
    rotation_F0_world2robot(1,1) = pow(anchor_orientation(3),2) - pow(anchor_orientation(0),2) + pow(anchor_orientation(1),2) - pow(anchor_orientation(2),2);
    rotation_F0_world2robot(1,2) = 2*(anchor_orientation(1)*anchor_orientation(2) + anchor_orientation(3)*anchor_orientation(0));
    rotation_F0_world2robot(2,0) = 2*(anchor_orientation(0)*anchor_orientation(2) + anchor_orientation(3)*anchor_orientation(1));
    rotation_F0_world2robot(2,1) = 2*(anchor_orientation(1)*anchor_orientation(2) - anchor_orientation(3)*anchor_orientation(0));
    rotation_F0_world2robot(2,2) = pow(anchor_orientation(3),2) - pow(anchor_orientation(0),2) - pow(anchor_orientation(1),2) + pow(anchor_orientation(2),2);

    Eigen::Matrix<T,3,3> rotation_robot2camera;
    rotation_robot2camera(0,0) = pow(sensor_orientation(3),2) + pow(sensor_orientation(0),2) - pow(sensor_orientation(1),2) - pow(sensor_orientation(2),2);
    rotation_robot2camera(0,1) = 2*(sensor_orientation(0)*sensor_orientation(1) + sensor_orientation(3)*sensor_orientation(2));
    rotation_robot2camera(0,2) = 2*(sensor_orientation(0)*sensor_orientation(2) - sensor_orientation(3)*sensor_orientation(1));;
    rotation_robot2camera(1,0) = 2*(sensor_orientation(0)*sensor_orientation(1) - sensor_orientation(3)*sensor_orientation(2));;
    rotation_robot2camera(1,1) = pow(sensor_orientation(3),2) - pow(sensor_orientation(0),2) + pow(sensor_orientation(1),2) - pow(sensor_orientation(2),2);
    rotation_robot2camera(1,2) = 2*(sensor_orientation(1)*sensor_orientation(2) + sensor_orientation(3)*sensor_orientation(0));
    rotation_robot2camera(2,0) = 2*(sensor_orientation(0)*sensor_orientation(2) + sensor_orientation(3)*sensor_orientation(1));
    rotation_robot2camera(2,1) = 2*(sensor_orientation(1)*sensor_orientation(2) - sensor_orientation(3)*sensor_orientation(0));
    rotation_robot2camera(2,2) = pow(sensor_orientation(3),2) - pow(sensor_orientation(0),2) - pow(sensor_orientation(1),2) + pow(sensor_orientation(2),2);
    /* end making the rotations */


    // ==================================================

    // camera in world coordinates

    Eigen::Matrix<T,3,3> rotation_c2w;
    rotation_c2w = rotation_F0_world2robot*rotation_robot2camera;

    Eigen::Matrix<T,3,1> translation_c2w;
    translation_c2w = (rotation_F0_world2robot*translation_robot2camera) + translation_F0_world2robot;

    // world in camera1 coordinates
    Eigen::Matrix<T,3,3> rotation_w2c1;
    rotation_w2c1 = rotation_robot2camera.transpose()*rotation_F1_world2robot.transpose();

    Eigen::Matrix<T,3,1> translation_w2c1;
    translation_w2c1 = (rotation_robot2camera.transpose()*(-rotation_F1_world2robot.transpose()*translation_F1_world2robot)) +
            (-rotation_robot2camera.transpose()*translation_robot2camera);

    // camera in camera1 coordinates, through world

    Eigen::Matrix<T,3,3> rotation_c2c1;
    rotation_c2c1 = rotation_w2c1 * rotation_c2w;

    Eigen::Matrix<T,3,1> translation_c2c1;
    translation_c2c1 = (rotation_w2c1 * translation_c2w) + translation_w2c1;

    //
    Eigen::Matrix<T,3,1> v;
    v = (rotation_c2c1 * m) + (translation_c2c1 * landmarkmap(3));
    std::cout << "v:\n" << v(0) << "\t" << v(1) << "\t" << v(2) << std::endl;
    // ==================================================

    Eigen::Matrix<T,3,1> u_;
    u_ = K * v;
    std::cout << "u_:\n" << u_(0) << "\t" << u_(1) << "\t" << u_(2) << std::endl;

    Eigen::Matrix<T,3,1> m2;
    m2 = rotation_c2c1*K.inverse()*u_;
    std::cout << "m2:\n" << m2(0) << "\t" << m2(1) << "\t" << m2(2) << std::endl;

    Eigen::Matrix<T,2,1> u_12;
    u_12(0) = u_(0);
    u_12(1) = u_(1);

    Eigen::Matrix<T,2,1> u;
    if(u_(2)!=0)
    {
        u = u_12 / u_(2);
        std::cout << "u_(2) != 0" << std::endl;
    }
    else
    {
        u = u_12;
        std::cout << "u_(2) == 0" << std::endl;
    }
        std::cout << "u:\n" << u(0) << "\t" << u(1) << std::endl;
    // ==================================================

    std::cout << "estimation of the projection: " << u.transpose() << std::endl;
    std::cout << "feature: " << feature_image_.getMeasurement().transpose() << std::endl;
    //Eigen::Matrix<T,2,1> feature_pos = getMeasurement().cast<T>();
    Eigen::Matrix<T,2,1> feature_pos = feature_image_.getMeasurement().cast<T>();

    residualsmap[0] = (u[0] - feature_pos[0]) * getMeasurementSquareRootInformation()(0, 0);
    residualsmap[1] = (u[1] - feature_pos[1]) * getMeasurementSquareRootInformation()(1, 1);

    std::cout << "\n\tRESIDUALS: \n" << residualsmap[0] << "\n" << residualsmap[1] << std::endl;

    // ==================================================

//    Eigen::Matrix<T,3,1> wTc0 = rotation_F1_world2robot*translation_robot2camera + translation_F1_world2robot;      // wRr*rTc + wTr*1
//    Eigen::Matrix<T,4,1> wRc1 = rotation_F1_world2robot*rotation_robot2camera;            // wRr*rRc + wTr*0
//    Eigen::Matrix<T,3,1> wTc1 = rotation_F1_world2robot*translation_robot2camera + translation_F1_world2robot;      // wRr*rTc + wTr*1


//    Eigen::Matrix<T,3,1> PI = K * wRc1.transpose() * (inverse_depth * (wTc0 - wTc1) + m);

//    Eigen::Matrix<T,2,1> PI_12;
//    PI_12(0) = PI(0);
//    PI_12(1) = PI(1);

//    Eigen::Matrix<T,2,1> landmark_estimation = PI_12/PI(3);


//    Eigen::Matrix<T,2,1> feature_pos = getMeasurement().cast<T>();

//    residualsmap[0] = (landmark_estimation[0] - feature_pos[0]) * getMeasurementSquareRootInformation()(0, 0);
//    residualsmap[1] = (landmark_estimation[1] - feature_pos[1]) * getMeasurementSquareRootInformation()(1, 1);

//    std::cout << "\n\tRESIDUALS: \n" << residualsmap[0] << "\n" << residualsmap[1] << std::endl;

    return true;
}
} // namespace wolf

#endif // CONSTRAINT_IMAGE_H
