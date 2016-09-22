#include "landmark_AHP.h"
#include "constraint_image.h"
#include "state_block.h"
#include "state_quaternion.h"
#include "sensor_camera.h"
#include "capture_image.h"

int main()
{
    using namespace wolf;

    std::cout << std::endl << "==================== constraint AHP test ======================" << std::endl;

    TimeStamp t = 1;

    char const* tmp = std::getenv( "WOLF_ROOT" );
    if ( tmp == nullptr )
        throw std::runtime_error("WOLF_ROOT environment not loaded.");
    std::string wolf_path( tmp );
    std::cout << "Wolf path: " << wolf_path << std::endl;

    // Wolf problem
    Problem* wolf_problem_ptr_ = new Problem(FRM_PO_3D);

    /* Do this while there aren't extrinsic parameters on the yaml */
    Eigen::Vector7s extrinsic_cam;
    extrinsic_cam[0] = 0; //px
    extrinsic_cam[1] = 3; //py
    extrinsic_cam[2] = 2; //pz
    extrinsic_cam[3] = 0; //qx
    extrinsic_cam[4] = 0; //qy
    extrinsic_cam[5] = 0; //qz
    extrinsic_cam[6] = 1; //qw
    std::cout << "========extrinsic_cam: " << extrinsic_cam.transpose() << std::endl;
    const Eigen::VectorXs extr = extrinsic_cam;
    /* Do this while there aren't extrinsic parameters on the yaml */

    SensorBase* sensor_ptr = wolf_problem_ptr_->installSensor("CAMERA", "PinHole", extr, wolf_path + "/src/examples/camera_params.yaml");
    SensorCamera* camera_ptr_ = (SensorCamera*)sensor_ptr;

    // PROCESSOR
    // one-liner API
    wolf_problem_ptr_->installProcessor("IMAGE LANDMARK", "ORB", "PinHole", wolf_path + "/src/examples/processor_image_ORB.yaml");


    // create the current frame
    Eigen::Vector7s frame_pos_ori;
    frame_pos_ori[0] = 0; //px
    frame_pos_ori[1] = 4; //py
    frame_pos_ori[2] = 2; //pz
    frame_pos_ori[3] = 0.2; //qx
    frame_pos_ori[4] = 0.5; //qy
    frame_pos_ori[5] = 0.1; //qz
    frame_pos_ori[6] = 0.83666; //qw
    const Eigen::VectorXs frame_val = frame_pos_ori;

    FrameBase* last_frame = new FrameBase(t,new StateBlock(frame_val.head(3)), new StateQuaternion(frame_val.tail(4)));
    std::cout << "Last frame" << std::endl;
    wolf_problem_ptr_->getTrajectoryPtr()->addFrame(last_frame);

    // Capture
    CaptureImage* image_ptr;
    t.setToNow();
    cv::Mat frame; //puede que necesite una imagen

    image_ptr = new CaptureImage(t, camera_ptr_, frame);
    wolf_problem_ptr_->getLastFramePtr()->addCapture(image_ptr);


    // create the feature
    cv::KeyPoint kp; kp.pt = {20,40};
    cv::Mat desc;

    FeaturePointImage* feat_point_image_ptr = new FeaturePointImage(kp, desc, Eigen::Matrix2s::Identity());
    image_ptr->addFeature(feat_point_image_ptr);

    // create the landmark
    Eigen::Vector3s point2D;
    point2D[0] = feat_point_image_ptr->getKeypoint().pt.x;
    point2D[1] = feat_point_image_ptr->getKeypoint().pt.y;
    point2D[2] = 1;

//    std::cout << "point2D x: " << point2D(0) << "; y: " << point2D(1) << "; z: " << point2D(2) << std::endl;
    Scalar depth = 2; // arbitrary value

    Eigen::Matrix3s K = ((SensorCamera*)(image_ptr->getSensorPtr()))->getIntrinsicMatrix();

    Eigen::Vector3s unitary_vector;
    unitary_vector = K.inverse() * point2D;
    unitary_vector.normalize();
//    std::cout << "unitary_vector: " << unitary_vector(0) << "\t" << unitary_vector(1) << "\t" << unitary_vector(2) << std::endl;

    FrameBase* anchor_frame = new FrameBase(t,new StateBlock(frame_val.head(3)), new StateQuaternion(frame_val.tail(4)));
    //FrameBase* anchor_frame = wolf_problem_ptr_->getTrajectoryPtr()->getLastFramePtr();



    /* DISTORTION ATTEMPT */
    Eigen::Vector3s test_undistortion;
    Eigen::VectorXs correction_vector = ((SensorCamera*)(image_ptr->getSensorPtr()))->getCorrectionVector();
    //test_distortion = pinhole::distortPoint(distortion_vector,test_distortion);
    //std::cout << "\ntest_point2D DISTORTED:\n" << test_distortion(0) << std::endl;


    Scalar r2 = unitary_vector(0) * unitary_vector(0) + unitary_vector(1) * unitary_vector(1); // this is the norm squared: r2 = ||u||^2
    //return distortFactor(d, r2) * up;


    Scalar s = 1.0;
    Scalar r2i = 1.0;
    //T i;
    //for (i = (T)0; i == (distortion_vector.cols()-1) ; i = i +(T)1) { //   here we are doing:
        r2i = r2i * r2;                                   //   r2i = r^(2*(i+1))
        s = s + (correction_vector(0) * r2i);             //   s = 1 + d_0 * r^2 + d_1 * r^4 + d_2 * r^6 + ...
        r2i = r2i * r2;
        s = s + (correction_vector(1) * r2i);
        r2i = r2i * r2;
        s = s + (correction_vector(2) * r2i);
        r2i = r2i * r2;
        s = s + (correction_vector(3) * r2i);
    //}
    if (s < 0.6) s = 1.0;
    test_undistortion(0) = s * unitary_vector(0);
    test_undistortion(1) = s * unitary_vector(1);
    test_undistortion(2) = unitary_vector(2);
    /* END OF THE ATTEMPT */



    Eigen::Vector4s vec_homogeneous = {test_undistortion(0),test_undistortion(1),test_undistortion(2),1/depth};

//    std::cout << "unitary_vec x: " << vec_homogeneous(0) << "; y: " << vec_homogeneous(1) << "; z: " << vec_homogeneous(2) << std::endl;

    LandmarkAHP* landmark = new LandmarkAHP(vec_homogeneous, anchor_frame, image_ptr->getSensorPtr(), feat_point_image_ptr->getDescriptor());

    std::cout << "Landmark AHP created" << std::endl;



    // Create the constraint
    ConstraintImage* constraint_ptr = new ConstraintImage(feat_point_image_ptr, last_frame,(LandmarkAHP*)landmark);

    feat_point_image_ptr->addConstraint(constraint_ptr);
    std::cout << "Constraint AHP created" << std::endl;



    Eigen::Vector2s residuals;
    Eigen::Vector3s current_frame_p = last_frame->getPPtr()->getVector();
    Eigen::Vector4s current_frame_o = last_frame->getOPtr()->getVector();
    Eigen::Vector3s anchor_frame_p = landmark->getAnchorFrame()->getPPtr()->getVector();
    Eigen::Vector4s anchor_frame_o = landmark->getAnchorFrame()->getOPtr()->getVector();
    Eigen::Vector4s landmark_ = landmark->getPPtr()->getVector();
    (*((ConstraintImage*) constraint_ptr))(current_frame_p.data(), current_frame_o.data(),
                                           anchor_frame_p.data(),anchor_frame_o.data(),
                                           landmark_.data(), residuals.data());
    // current frame p; current frame o; anchor frame p; anchor frame o; homogeneous vector landmark, residual


    std::cout << "Residual computed" << std::endl;
    std::cout << "Residual = " << residuals[0] << "   " << residuals[1] << std::endl;


    delete wolf_problem_ptr_;

    return 0;

}
