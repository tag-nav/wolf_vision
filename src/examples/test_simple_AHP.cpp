/**
 * \file test_simple_AHP.cpp
 *
 *  Created on: 2 nov. 2016
 *      \author: jtarraso
 */

#include "wolf.h"
#include "frame_base.h"
#include "sensor_camera.h"
#include "pinholeTools.h"
#include "rotations.h"
#include "capture_image.h"
#include "landmark_AHP.h"
#include "constraint_AHP.h"
#include "ceres_wrapper/ceres_manager.h"


int main(int argc, char** argv)
{
    using namespace wolf;
    using namespace Eigen;

    /* 1. crear 2 kf, fixed
     * 2. crear 1 sensor
     * 3. crear 1 lmk1
     * 4. projectar lmk sobre sensor a fk1
     * 5. projectar lmk sobre sensor a kf2
     * 6. // esborrar lmk lmk_ptr->remove() no cal
     * 7. crear nous kf
     * 8. crear captures
     * 9. crear features amb les mesures de 4 i 5
     * 10. crear lmk2 des de kf1
     * 11. crear constraint des del kf2 a lmk2, amb ancora al kf1
     * 12. solve
     * 13. lmk1 == lmk2 ?
     */

    // ============================================================================================================
    /* 1 */
    ProblemPtr wolf_problem_ptr_ = Problem::create(FRM_PO_3D);
    FrameBasePtr kf1 = wolf_problem_ptr_->createFrame(KEY_FRAME,(Vector7s()<<0,0,0,0,0,0,1).finished(), TimeStamp(0));
    FrameBasePtr kf2 = wolf_problem_ptr_->createFrame(KEY_FRAME,(Vector7s()<<0,0.5,0,0,0,0,1).finished(), TimeStamp(0));

    kf1->fix();
    kf2->fix();
    // ============================================================================================================

    // ============================================================================================================
    /* 2 */
    GET_WOLF_ROOT
    SensorBasePtr sensor_ptr = wolf_problem_ptr_->installSensor("CAMERA", "PinHole", (Vector7s()<<0,0,0,-0.5,0.5,-0.5,0.5).finished(), wolf_root + "/src/examples/camera_params_ueye_sim.yaml");
    SensorCamera::Ptr camera_ptr = std::static_pointer_cast<SensorCamera>(sensor_ptr);
    // ============================================================================================================

    // ============================================================================================================
    /* 3 */
    Eigen::Vector3s lmk_p3D = {2,0,0};
    std::cout << std::endl << "lmk: " << lmk_p3D.transpose() << std::endl;
    lmk_p3D.normalize();
    Eigen::Vector4s lmk_hmg_w;
    Scalar distance = 2;
    lmk_hmg_w = {lmk_p3D(0),lmk_p3D(1),lmk_p3D(2),(1/distance)};
//    std::cout << "lmk hmg in w: " << lmk_hmg_w.transpose() << std::endl;
    // ============================================================================================================


    // Captures
    cv::Mat frame;
    frame.zeros(2,2,0);
    CaptureImage::Ptr image_ptr_1 = std::make_shared<CaptureImage>(TimeStamp(0), camera_ptr, frame);
    CaptureImage::Ptr image_ptr_2 = std::make_shared<CaptureImage>(TimeStamp(0), camera_ptr, frame);
    kf1->addCapture(image_ptr_1);
    kf2->addCapture(image_ptr_2);

    // Features
    cv::KeyPoint kp1; kp1.pt.x = 0; kp1.pt.y = 0;
    cv::KeyPoint kp2; kp2.pt.x = 0; kp2.pt.y = 0;
    cv::Mat desc;

    std::shared_ptr<FeaturePointImage> feat_point_image_ptr_1 = std::make_shared<FeaturePointImage>(kp1, desc, Eigen::Matrix2s::Identity());
    image_ptr_1->addFeature(feat_point_image_ptr_1);

    std::shared_ptr<FeaturePointImage> feat_point_image_ptr_2 = std::make_shared<FeaturePointImage>(kp2, desc, Eigen::Matrix2s::Identity());
    image_ptr_2->addFeature(feat_point_image_ptr_2);

    // Landmark
    std::shared_ptr<LandmarkAHP> lmk_ahp_ptr1 = std::make_shared<LandmarkAHP>(lmk_hmg_w, kf1, camera_ptr, desc);

    // Constraints
    ConstraintAHP::Ptr constraint_ptr1 = std::make_shared<ConstraintAHP>(feat_point_image_ptr_1, kf1, lmk_ahp_ptr1 );
    feat_point_image_ptr_2->addConstraint(constraint_ptr1);

    Eigen::Vector2s pix = constraint_ptr1->expectation();




















    // ============================================================================================================
    /* 4 */

    /* change the reference from world to camera (kf1) */
    Eigen::Vector3s pwr1 = kf1->getPPtr()->getVector();
    Eigen::Vector3s prc = wolf_problem_ptr_->getSensorPtr("PinHole")->getPPtr()->getVector();
//    std::cout << "keyframe pose: " << pwr1.transpose() << std::endl;
//    std::cout << "camera pose:   " << prc.transpose() << std::endl;

    Eigen::Translation<Scalar,3> twr1, trc;
    twr1.x() = pwr1(0); twr1.y() = pwr1(1); twr1.z() = pwr1(2);
    trc.x() = prc(0); trc.y() = prc(1); trc.z() = prc(2);

    Eigen::Quaternion<Scalar> qwr1, qwr0, qrc;
    Eigen::Vector4s quaternion_current_frame = kf1->getOPtr()->getVector();
    Eigen::Vector4s quaternion_sensor = wolf_problem_ptr_->getSensorPtr("PinHole")->getOPtr()->getVector();
    qwr1 = quaternion_current_frame;
    qrc = quaternion_sensor;
//    std::cout << "keyframe orientation: " << qwr1.vec().transpose() << " " << qwr1.w() << std::endl;
//    std::cout << "camera orientation:   " << qrc.vec().transpose() <<  " " << qrc.w() << std::endl;

    Eigen::Transform<Scalar,3,Eigen::Affine> T_W_R1, T_R1_C1;
    T_W_R1 = twr1 * qwr1;
    T_R1_C1 = trc * qrc;

    Eigen::Vector4s lmk1_hmg_c1;
    lmk1_hmg_c1 = T_R1_C1.inverse(Eigen::Affine) * T_W_R1.inverse(Eigen::Affine) * lmk_hmg_w;
//    std::cout << "lmk hmg in c1: " << lmk1_hmg_c1.transpose() << std::endl;

    /* project the landmark (kf1) */
    Eigen::Vector3s point2D_hmg = lmk1_hmg_c1.head(3);
    Eigen::Vector2s point2D_kf1 = point2D_hmg.head(2)/point2D_hmg(2);
    point2D_kf1 = pinhole::distortPoint((std::static_pointer_cast<SensorCamera>(wolf_problem_ptr_->getSensorPtr("PinHole")))->getDistortionVector(),point2D_kf1);
    point2D_kf1 = pinhole::pixellizePoint(wolf_problem_ptr_->getSensorPtr("PinHole")->getIntrinsicPtr()->getVector(),point2D_kf1);

    std::cout << "2D point (kf1): " << point2D_kf1.transpose() << std::endl;

//    /* to test the projection - backprojection */
//    point2D_kf1 = pinhole::depixellizePoint(wolf_problem_ptr_->getSensorPtr("PinHole")->getIntrinsicPtr()->getVector(),point2D_kf1);
//    point2D_kf1 = pinhole::undistortPoint((std::static_pointer_cast<SensorCamera>(wolf_problem_ptr_->getSensorPtr("PinHole")))->getCorrectionVector(),point2D_kf1);
//
//    Eigen::Vector3s point3D;
//    point3D.head<2>() = point2D_kf1;
//    point3D(2) = 1;
//    point3D.normalize();
//
//    Eigen::Vector4s point3D_hmg;
//    point3D_hmg = {point3D(0),point3D(1),point3D(2),1/distance};
//
//    Eigen::Vector4s lmk1_hmg_bp_w;
//    lmk1_hmg_bp_w = T_W_R1 * T_R1_C1 * point3D_hmg;
//
//    std::cout << "3D hmg point: " << (lmk1_hmg_bp_w.head(3)/(1/distance)).transpose() << std::endl;

    // ============================================================================================================

    // ============================================================================================================
    /* 5 */

    /* change the reference from world to camera (kf2) */
    Eigen::Vector3s pwr1_kf2 = kf2->getPPtr()->getVector();
    Eigen::Translation<Scalar,3> twr1_;
    twr1_.x() = pwr1_kf2(0); twr1_.y() = pwr1_kf2(1); twr1_.z() = pwr1_kf2(2);
//        std::cout << "keyframe pose: " << pwr1_kf2.transpose() << std::endl;
//        std::cout << "camera pose:   " << prc.transpose() << std::endl;

    Eigen::Quaternion<Scalar> qwr1_kf2;
    Eigen::Vector4s quaternion_current_frame_kf2 = kf2->getOPtr()->getVector();
    qwr1_kf2 = quaternion_current_frame_kf2;
//    std::cout << "keyframe orientation: " << qwr1_kf2.vec().transpose() << " " << qwr1_kf2.w() << std::endl;
//    std::cout << "camera orientation:   " << qrc.vec().transpose() <<  " " << qrc.w() << std::endl;


    Eigen::Transform<Scalar,3,Eigen::Affine> T_W_R1_;
    T_W_R1_ = twr1_ * qwr1_kf2;

    Eigen::Vector4s lmk2_hmg_c1;
//    std::cout << "lmk hmg in w: " << lmk_hmg_w.transpose() << std::endl;
    lmk2_hmg_c1 = T_R1_C1.inverse(Eigen::Affine) * T_W_R1_.inverse(Eigen::Affine) * lmk_hmg_w;
//    std::cout << "lmk hmg in c1: " << lmk2_hmg_c1.transpose() << std::endl;

    /* project the landmark (kf2) */
    Eigen::Vector3s point2D_hmg_fk2 = lmk2_hmg_c1.head(3);
    Eigen::Vector2s point2D_kf2 = point2D_hmg_fk2.head(2)/point2D_hmg_fk2(2);
    point2D_kf2 = pinhole::distortPoint((std::static_pointer_cast<SensorCamera>(wolf_problem_ptr_->getSensorPtr("PinHole")))->getDistortionVector(),point2D_kf2);
    point2D_kf2 = pinhole::pixellizePoint(wolf_problem_ptr_->getSensorPtr("PinHole")->getIntrinsicPtr()->getVector(),point2D_kf2);

    std::cout << "2D point (kf2): " << point2D_kf2.transpose() << std::endl;

    /* to test the projection - backprojection */
    point2D_kf2 = pinhole::depixellizePoint(wolf_problem_ptr_->getSensorPtr("PinHole")->getIntrinsicPtr()->getVector(),point2D_kf2);
    point2D_kf2 = pinhole::undistortPoint((std::static_pointer_cast<SensorCamera>(wolf_problem_ptr_->getSensorPtr("PinHole")))->getCorrectionVector(),point2D_kf2);

    Eigen::Vector3s point3D;
    point3D.head<2>() = point2D_kf2;
    point3D(2) = 1;
    point3D.normalize();

    Eigen::Vector4s point3D_hmg;
    point3D_hmg = {point3D(0),point3D(1),point3D(2),1/distance};

    Eigen::Vector4s lmk1_hmg_bp_w;
    lmk1_hmg_bp_w = T_W_R1_ * T_R1_C1 * point3D_hmg;

    std::cout << "3D hmg point (kf2): " << (lmk1_hmg_bp_w.head(3)/(1/distance)).transpose() << std::endl;

    // ============================================================================================================

    // ============================================================================================================
    /* 7 */
    FrameBasePtr kf3 = wolf_problem_ptr_->createFrame(KEY_FRAME,(Vector7s()<<0,0,0,0,0,0,1).finished(), TimeStamp(0));
    FrameBasePtr kf4 = wolf_problem_ptr_->createFrame(KEY_FRAME,(Vector7s()<<0,0.5,0,0,0,0,1).finished(), TimeStamp(0));

    // ============================================================================================================

    // ============================================================================================================
    /* 8 */
    cv::Mat frame;
    frame.zeros(2,2,0);
    CaptureImage::Ptr image_ptr_1 = std::make_shared<CaptureImage>(TimeStamp(0), camera_ptr, frame);
    CaptureImage::Ptr image_ptr_2 = std::make_shared<CaptureImage>(TimeStamp(0), camera_ptr, frame);
    kf3->addCapture(image_ptr_1);
    kf4->addCapture(image_ptr_2);
    // ============================================================================================================

    // ============================================================================================================
    /* 9 */
    cv::KeyPoint kp1; kp1.pt.x = point2D_kf1(0); kp1.pt.y = point2D_kf1(1);
    cv::KeyPoint kp2; kp2.pt.x = point2D_kf2(0); kp2.pt.y = point2D_kf2(1);
    cv::Mat desc;

    std::shared_ptr<FeaturePointImage> feat_point_image_ptr_1 = std::make_shared<FeaturePointImage>(kp1, desc, Eigen::Matrix2s::Identity());
    image_ptr_1->addFeature(feat_point_image_ptr_1);

    std::shared_ptr<FeaturePointImage> feat_point_image_ptr_2 = std::make_shared<FeaturePointImage>(kp2, desc, Eigen::Matrix2s::Identity());
    image_ptr_2->addFeature(feat_point_image_ptr_2);
    // ============================================================================================================

    // ============================================================================================================
    /* 10 */
    /* to test the projection - backprojection */
    Eigen::Vector2s p2D_kf3;
    p2D_kf3(0)= feat_point_image_ptr_1->getKeypoint().pt.x;
    p2D_kf3(1)= feat_point_image_ptr_1->getKeypoint().pt.y;
    Eigen::Vector2s point2D_lmk = pinhole::depixellizePoint(wolf_problem_ptr_->getSensorPtr("PinHole")->getIntrinsicPtr()->getVector(), p2D_kf3);
    point2D_lmk = pinhole::undistortPoint((std::static_pointer_cast<SensorCamera>(wolf_problem_ptr_->getSensorPtr("PinHole")))->getCorrectionVector(),point2D_lmk);

    Eigen::Vector3s point3D_kf3;
    point3D_kf3.head<2>() = point2D_lmk;
    point3D_kf3(2) = 1;
    point3D_kf3.normalize();

    Eigen::Vector4s point3D_hmg_kf3;
    point3D_hmg_kf3 = {point3D_kf3(0),point3D_kf3(1),point3D_kf3(2),1/distance};

    /* change the reference from world to camera (kf2) */
    Eigen::Vector3s pwr1_kf3 = kf3->getPPtr()->getVector();
    Eigen::Translation<Scalar,3> twr1_kf3;
    twr1_kf3.x() = pwr1_kf3(0); twr1_kf3.y() = pwr1_kf3(1); twr1_kf3.z() = pwr1_kf3(2);
    std::cout << "keyframe pose: " << pwr1_kf3.transpose() << std::endl;

    Eigen::Quaternion<Scalar> qwr1_kf3;
    Eigen::Vector4s quaternion_current_frame_kf3 = kf3->getOPtr()->getVector();
    qwr1_kf3 = quaternion_current_frame_kf3;
    std::cout << "keyframe orientation: " << qwr1_kf3.vec().transpose() << " " << qwr1_kf3.w() << std::endl;


    Eigen::Transform<Scalar,3,Eigen::Affine> T_W_R1_kf3;
    T_W_R1_kf3 = twr1_kf3 * qwr1_kf3;

    Eigen::Vector4s lmk1_hmg_kf3_w;
    lmk1_hmg_kf3_w = T_W_R1_kf3 * T_R1_C1 * point3D_hmg_kf3;

    std::cout << "3D hmg point: " << (lmk1_hmg_kf3_w.head(3)/(1/distance)).transpose() << std::endl;

    std::shared_ptr<LandmarkAHP> lmk_ahp_ptr = std::make_shared<LandmarkAHP>(lmk1_hmg_kf3_w, kf3, camera_ptr, feat_point_image_ptr_1->getDescriptor());
    // ============================================================================================================

    // ============================================================================================================
    /* 11 */
    ConstraintAHP::Ptr constraint_ptr = std::make_shared<ConstraintAHP>(feat_point_image_ptr_2, kf4, lmk_ahp_ptr );
    feat_point_image_ptr_2->addConstraint(constraint_ptr);

    // ============================================================================================================

    // ============================================================================================================
    /* 12 */
    // Ceres wrapper
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    //    ceres_options.minimizer_progress_to_stdout = false;
    //    ceres_options.line_search_direction_type = ceres::LBFGS;
    //    ceres_options.max_num_iterations = 100;
    google::InitGoogleLogging(argv[0]);

    CeresManager ceres_manager(wolf_problem_ptr_, ceres_options);


    ceres::Solver::Summary summary = ceres_manager.solve();
    std::cout << summary.FullReport() << std::endl;

}
