#ifndef CONSTRAINT_IMAGE_NEW_LANDMARK_H
#define CONSTRAINT_IMAGE_NEW_LANDMARK_H

//Wolf includes
#include "constraint_sparse.h"
#include "landmark_AHP.h"
#include "sensor_camera.h"
#include "pinholeTools.h"
#include "feature_point_image.h"

namespace wolf {

class ConstraintImageNewLandmark : public ConstraintSparse<2, 3, 4, 4>
{
    protected:
        Eigen::Vector4s intrinsics_;
        Eigen::Vector3s extrinsics_p_;
        Eigen::Vector4s extrinsics_o_;
        FeaturePointImage feature_image_;

    public:
        static const unsigned int N_BLOCKS = 3; //TODO: Prueba a comentarlo

        ConstraintImageNewLandmark(FeatureBase* _ftr_ptr, FrameBase* _frame_ptr, LandmarkAHP* _landmark_ptr,
                        bool _apply_loss_function = false, ConstraintStatus _status = CTR_ACTIVE) :
                ConstraintSparse<2, 3, 4, 4>(CTR_EPIPOLAR_NL, _landmark_ptr, _apply_loss_function, _status,
                                             _frame_ptr->getPPtr(), _frame_ptr->getOPtr(),_landmark_ptr->getPPtr()),
                intrinsics_(_ftr_ptr->getCapturePtr()->getSensorPtr()->getIntrinsicPtr()->getVector()),
                extrinsics_p_(_ftr_ptr->getCapturePtr()->getSensorPPtr()->getVector()),
                extrinsics_o_(_ftr_ptr->getCapturePtr()->getSensorOPtr()->getVector()),
                feature_image_(*((FeaturePointImage*)_ftr_ptr))
        {
            setType("IMAGE");

        }

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~ConstraintImageNewLandmark()
        {
            //
        }

        template<typename T>
        bool operator ()(const T* const _probot, const T* const _orobot,const T* const _lmk, T* _residuals) const;

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
inline bool ConstraintImageNewLandmark::operator ()(const T* const _p_robot, const T* const _o_robot, const T* const _p_lmk,
                                         T* _residuals) const
{
    // Do the magic here
    Eigen::Map<const Eigen::Matrix<T, 3, 1> > translation_F1_world2robot(_p_robot); //translation_world2robot
    Eigen::Map<const Eigen::Matrix<T, 4, 1> > orobotmap(_o_robot);
    Eigen::Map<Eigen::Matrix<T, 2, 1> > residualsmap(_residuals);

//    std::cout << "probot:\n" << translation_F1_world2robot(0) << "\t" << translation_F1_world2robot(1)
//              << "\t" << translation_F1_world2robot(2) << std::endl;
//    std::cout << "orobot:\n" << orobotmap(0) << "\t" << orobotmap(1) << "\t" << orobotmap(2) << "\t" << orobotmap(3)<< std::endl;
//    std::cout << "residuals:\n" << residualsmap(0) << "\t" << residualsmap(1) << std::endl;

    Eigen::Matrix<T,4,1> k_params = intrinsics_.cast<T>();
    Eigen::Matrix<T,3,3> K;
    K(0,0) = k_params(2);
    K(0,1) = T(0);
    K(0,2) = k_params(0);
    K(1,0) = T(0);
    K(1,1) = k_params(3);
    K(1,2) = k_params(1);
    K(2,0) = T(0);
    K(2,1) = T(0);
    K(2,2) = T(1);

//    std::cout << "K matrix:\n" << K(0,0) << "\t" << K(0,1) << "\t" << K(0,2) << "\n"
//              << K(1,0) << "\t" << K(1,1) << "\t" << K(1,2) << "\n"
//              << K(2,0) << "\t" << K(2,1) << "\t" << K(2,2) << "\n" << std::endl;

    Eigen::Matrix<T,3,1> translation_robot2camera = extrinsics_p_.cast<T>();
    Eigen::Matrix<T,4,1> sensor_orientation = extrinsics_o_.cast<T>();
//    std::cout << "translation robot to camera:\n" << translation_robot2camera(0) << "\t" << translation_robot2camera(1)
//              << "\t" << translation_robot2camera(2) << std::endl;
//    std::cout << "orientation robot to camera:\n" << sensor_orientation(0) << "\t" << sensor_orientation(1) << "\t"
//              << sensor_orientation(2) << "\t" << sensor_orientation(3)<< std::endl;


    Eigen::Map<const Eigen::Matrix<T, 3, 1> > translation_F0_world2robot(_p_robot);
    Eigen::Map<const Eigen::Matrix<T, 4, 1> > anchor_orientation(_o_robot);

//    std::cout << "translation F0 world to robot:\n" << translation_F0_world2robot(0) << "\t" << translation_F0_world2robot(1)
//              << "\t" << translation_F0_world2robot(2) << std::endl;
//    std::cout << "orientation F0 world to robot:\n" << anchor_orientation(0) << "\t" << anchor_orientation(1) << "\t"
//              << anchor_orientation(2) << "\t" << anchor_orientation(3)<< std::endl;


    Eigen::Map<const Eigen::Matrix<T, 4, 1> > landmarkmap(_p_lmk);
    //Eigen::Matrix<T,1,1> inverse_depth;
    Eigen::Matrix<T,3,1> m;
    m(0) = landmarkmap(0);
    m(1) = landmarkmap(1);
    m(2) = landmarkmap(2);
//    std::cout << "m:\n" << m(0) << "\t" << m(1) << "\t" << m(2) << "\ninverse depth: " << landmarkmap(3) << std::endl;

    /* making the rotations manually now */
    Eigen::Matrix<T,3,3> rotation_F1_world2robot;
    rotation_F1_world2robot(0,0) = pow(orobotmap(3),2) + pow(orobotmap(0),2) - pow(orobotmap(1),2) - pow(orobotmap(2),2);
    rotation_F1_world2robot(0,1) = T(2)*(orobotmap(0)*orobotmap(1) + orobotmap(3)*orobotmap(2));
    rotation_F1_world2robot(0,2) = T(2)*(orobotmap(0)*orobotmap(2) - orobotmap(3)*orobotmap(1));;
    rotation_F1_world2robot(1,0) = T(2)*(orobotmap(0)*orobotmap(1) - orobotmap(3)*orobotmap(2));;
    rotation_F1_world2robot(1,1) = pow(orobotmap(3),2) - pow(orobotmap(0),2) + pow(orobotmap(1),2) - pow(orobotmap(2),2);
    rotation_F1_world2robot(1,2) = T(2)*(orobotmap(1)*orobotmap(2) + orobotmap(3)*orobotmap(0));
    rotation_F1_world2robot(2,0) = T(2)*(orobotmap(0)*orobotmap(2) + orobotmap(3)*orobotmap(1));
    rotation_F1_world2robot(2,1) = T(2)*(orobotmap(1)*orobotmap(2) - orobotmap(3)*orobotmap(0));
    rotation_F1_world2robot(2,2) = pow(orobotmap(3),2) - pow(orobotmap(0),2) - pow(orobotmap(1),2) + pow(orobotmap(2),2);

//    std::cout << "\nrotation F1 world to robot:\n"
//              << rotation_F1_world2robot(0,0) << "\t" << rotation_F1_world2robot(0,1) << "\t" << rotation_F1_world2robot(0,2) << "\n"
//              << rotation_F1_world2robot(1,0) << "\t" << rotation_F1_world2robot(1,1) << "\t" << rotation_F1_world2robot(1,2) << "\n"
//              << rotation_F1_world2robot(2,0) << "\t" << rotation_F1_world2robot(2,1) << "\t" << rotation_F1_world2robot(2,2) << "\n\n";

    Eigen::Matrix<T,3,3> rotation_F0_world2robot;
    rotation_F0_world2robot(0,0) = pow(anchor_orientation(3),2) + pow(anchor_orientation(0),2) - pow(anchor_orientation(1),2) - pow(anchor_orientation(2),2);
    rotation_F0_world2robot(0,1) = T(2)*(anchor_orientation(0)*anchor_orientation(1) + anchor_orientation(3)*anchor_orientation(2));
    rotation_F0_world2robot(0,2) = T(2)*(anchor_orientation(0)*anchor_orientation(2) - anchor_orientation(3)*anchor_orientation(1));;
    rotation_F0_world2robot(1,0) = T(2)*(anchor_orientation(0)*anchor_orientation(1) - anchor_orientation(3)*anchor_orientation(2));;
    rotation_F0_world2robot(1,1) = pow(anchor_orientation(3),2) - pow(anchor_orientation(0),2) + pow(anchor_orientation(1),2) - pow(anchor_orientation(2),2);
    rotation_F0_world2robot(1,2) = T(2)*(anchor_orientation(1)*anchor_orientation(2) + anchor_orientation(3)*anchor_orientation(0));
    rotation_F0_world2robot(2,0) = T(2)*(anchor_orientation(0)*anchor_orientation(2) + anchor_orientation(3)*anchor_orientation(1));
    rotation_F0_world2robot(2,1) = T(2)*(anchor_orientation(1)*anchor_orientation(2) - anchor_orientation(3)*anchor_orientation(0));
    rotation_F0_world2robot(2,2) = pow(anchor_orientation(3),2) - pow(anchor_orientation(0),2) - pow(anchor_orientation(1),2) + pow(anchor_orientation(2),2);

//    std::cout << "\nrotation F0 world to robot:\n"
//              << rotation_F0_world2robot(0,0) << "\t" << rotation_F0_world2robot(0,1) << "\t" << rotation_F0_world2robot(0,2) << "\n"
//              << rotation_F0_world2robot(1,0) << "\t" << rotation_F0_world2robot(1,1) << "\t" << rotation_F0_world2robot(1,2) << "\n"
//              << rotation_F0_world2robot(2,0) << "\t" << rotation_F0_world2robot(2,1) << "\t" << rotation_F0_world2robot(2,2) << "\n\n";

    Eigen::Matrix<T,3,3> rotation_robot2camera;
    rotation_robot2camera(0,0) = pow(sensor_orientation(3),2) + pow(sensor_orientation(0),2) - pow(sensor_orientation(1),2) - pow(sensor_orientation(2),2);
    rotation_robot2camera(0,1) = T(2)*(sensor_orientation(0)*sensor_orientation(1) + sensor_orientation(3)*sensor_orientation(2));
    rotation_robot2camera(0,2) = T(2)*(sensor_orientation(0)*sensor_orientation(2) - sensor_orientation(3)*sensor_orientation(1));;
    rotation_robot2camera(1,0) = T(2)*(sensor_orientation(0)*sensor_orientation(1) - sensor_orientation(3)*sensor_orientation(2));;
    rotation_robot2camera(1,1) = pow(sensor_orientation(3),2) - pow(sensor_orientation(0),2) + pow(sensor_orientation(1),2) - pow(sensor_orientation(2),2);
    rotation_robot2camera(1,2) = T(2)*(sensor_orientation(1)*sensor_orientation(2) + sensor_orientation(3)*sensor_orientation(0));
    rotation_robot2camera(2,0) = T(2)*(sensor_orientation(0)*sensor_orientation(2) + sensor_orientation(3)*sensor_orientation(1));
    rotation_robot2camera(2,1) = T(2)*(sensor_orientation(1)*sensor_orientation(2) - sensor_orientation(3)*sensor_orientation(0));
    rotation_robot2camera(2,2) = pow(sensor_orientation(3),2) - pow(sensor_orientation(0),2) - pow(sensor_orientation(1),2) + pow(sensor_orientation(2),2);

//    std::cout << "\nrotation robot to camera:\n"
//              << rotation_robot2camera(0,0) << "\t" << rotation_robot2camera(0,1) << "\t" << rotation_robot2camera(0,2) << "\n"
//              << rotation_robot2camera(1,0) << "\t" << rotation_robot2camera(1,1) << "\t" << rotation_robot2camera(1,2) << "\n"
//              << rotation_robot2camera(2,0) << "\t" << rotation_robot2camera(2,1) << "\t" << rotation_robot2camera(2,2) << "\n\n";

    /* end making the rotations */


    // ==================================================

    // camera in world coordinates

    Eigen::Matrix<T,3,3> rotation_c2w;
    rotation_c2w = rotation_F0_world2robot*rotation_robot2camera;

//    std::cout << "\nrotation_c2w:\n"
//              << rotation_c2w(0,0) << "\t" << rotation_c2w(0,1) << "\t" << rotation_c2w(0,2) << "\n"
//              << rotation_c2w(1,0) << "\t" << rotation_c2w(1,1) << "\t" << rotation_c2w(1,2) << "\n"
//              << rotation_c2w(2,0) << "\t" << rotation_c2w(2,1) << "\t" << rotation_c2w(2,2) << "\n";

    Eigen::Matrix<T,3,1> translation_c2w;
    translation_c2w = (rotation_F0_world2robot*translation_robot2camera) + translation_F0_world2robot;

//    std::cout << "\ntranslation_c2w:\n" << translation_c2w(0) << "\t" << translation_c2w(1) << "\t" << translation_c2w(2) << std::endl;

    // world in camera1 coordinates
    Eigen::Matrix<T,3,3> rotation_w2c1;
    rotation_w2c1 = rotation_robot2camera.transpose()*rotation_F1_world2robot.transpose();

//    std::cout << "\nrotation_w2c1:\n"
//              << rotation_w2c1(0,0) << "\t" << rotation_w2c1(0,1) << "\t" << rotation_w2c1(0,2) << "\n"
//              << rotation_w2c1(1,0) << "\t" << rotation_w2c1(1,1) << "\t" << rotation_w2c1(1,2) << "\n"
//              << rotation_w2c1(2,0) << "\t" << rotation_w2c1(2,1) << "\t" << rotation_w2c1(2,2) << "\n";

    Eigen::Matrix<T,3,1> translation_w2c1;
    translation_w2c1 = (rotation_robot2camera.transpose()*(-rotation_F1_world2robot.transpose()*translation_F1_world2robot)) +
            (-rotation_robot2camera.transpose()*translation_robot2camera);

//    std::cout << "\ntranslation_w2c1:\n" << translation_w2c1(0) << "\t" << translation_w2c1(1) << "\t" << translation_w2c1(2) << std::endl;

    // camera in camera1 coordinates, through world
    Eigen::Matrix<T,3,3> rotation_c2c1;
    rotation_c2c1 = rotation_w2c1 * rotation_c2w;

    std::cout << "\nrotation_c2c1:\n"
              << rotation_c2c1(0,0) << "\t" << rotation_c2c1(0,1) << "\t" << rotation_c2c1(0,2) << "\n"
              << rotation_c2c1(1,0) << "\t" << rotation_c2c1(1,1) << "\t" << rotation_c2c1(1,2) << "\n"
              << rotation_c2c1(2,0) << "\t" << rotation_c2c1(2,1) << "\t" << rotation_c2c1(2,2) << "\n";

    Eigen::Matrix<T,3,1> translation_c2c1;
    translation_c2c1 = (rotation_w2c1 * translation_c2w) + translation_w2c1;

    std::cout << "\ntranslation_c2c1:\n" << translation_c2c1(0) << "\t" << translation_c2c1(1) << "\t" << translation_c2c1(2) << std::endl;

    //
    Eigen::Matrix<T,3,1> v;
    v = (rotation_c2c1 * m) + (translation_c2c1 * landmarkmap(3));
    std::cout << "\nv:\n" << v(0) << "\t" << v(1) << "\t" << v(2) << std::endl;
    // ==================================================

    Eigen::Matrix<T,3,1> u_;
    u_ = K * v;
    std::cout << "\nu_:\n" << u_(0) << "\t" << u_(1) << "\t" << u_(2) << std::endl;

//    Eigen::Matrix<T,3,1> m2;
//    m2 = rotation_c2c1*K.inverse()*u_;
//    std::cout << "m2:\n" << m2(0) << "\t" << m2(1) << "\t" << m2(2) << std::endl;

    Eigen::Matrix<T,2,1> u_12;
    u_12(0) = u_(0);
    u_12(1) = u_(1);

    Eigen::Matrix<T,2,1> u;
    if(u_(2)!=T(0))
    {
        u = u_12 / u_(2);
        //std::cout << "u_(2) != 0" << std::endl;
    }
    else
    {
        u = u_12;
        //std::cout << "u_(2) == 0" << std::endl;
    }
        std::cout << "\nu:\n" << u(0) << "\t" << u(1) << std::endl;
    // ==================================================
    std::cout << "==============================================\nCONSTRAINT INFO" << std::endl;
    std::cout << "Estimation of the Projection:\n\t" << u(0) << "\n\t" << u(1) << std::endl;
    std::cout << "Feature measurement:\n" << getMeasurement() << std::endl;
    Eigen::Matrix<T,2,1> feature_pos = getMeasurement().cast<T>();
    //Eigen::Matrix<T,2,1> feature_pos = feature_image_.getMeasurement().cast<T>();

//    std::cout << "measurement square root information" << getMeasurementSquareRootInformation()(0, 0) << std::endl;

//    residualsmap[0] = (u[0] - feature_pos[0]) * getMeasurementSquareRootInformation()(0, 0);
//    residualsmap[1] = (u[1] - feature_pos[1]) * getMeasurementSquareRootInformation()(1, 1);

    residualsmap = getMeasurementSquareRootInformation().cast<T>() * (u - feature_pos);

    std::cout << "\nRESIDUALS: \n\t" << residualsmap[0] << "\n\t" << residualsmap[1] << std::endl;

    return true;
}
} // namespace wolf

#endif // CONSTRAINT_IMAGE_NEW_LANDMARK_H
