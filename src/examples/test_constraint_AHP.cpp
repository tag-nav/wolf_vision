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
    extrinsic_cam[1] = 0; //py
    extrinsic_cam[2] = 0; //pz
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
    frame_pos_ori[1] = 0; //py
    frame_pos_ori[2] = 0; //pz
    frame_pos_ori[3] = 0; //qx
    frame_pos_ori[4] = 0; //qy
    frame_pos_ori[5] = 0; //qz
    frame_pos_ori[6] = 1; //qw
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
    cv::KeyPoint kp; kp.pt = {5,5};
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

    Eigen::Vector4s k_params = image_ptr->getSensorPtr()->getIntrinsicPtr()->getVector();
    Eigen::Matrix3s K;
    K(0,0) = k_params(2);       K(0,1) = 0;                    K(0,2) = k_params(0);
    K(1,0) = 0;                 K(1,1) = k_params(3);          K(1,2) = k_params(1);
    K(2,0) = 0;                 K(2,1) = 0;                    K(2,2) = 1;

//    std::cout << "K: " << k_params << std::endl;

    Eigen::Vector3s unitary_vector;
    unitary_vector = K.inverse() * point2D;
    unitary_vector.normalize();
//    std::cout << "unitary_vector: " << unitary_vector(0) << "\t" << unitary_vector(1) << "\t" << unitary_vector(2) << std::endl;


    FrameBase* anchor_frame = new FrameBase(t,new StateBlock(frame_val.head(3)), new StateQuaternion(frame_val.tail(4)));
    //FrameBase* anchor_frame = wolf_problem_ptr_->getTrajectoryPtr()->getLastFramePtr();

    Eigen::Vector4s vec_homogeneous = {unitary_vector(0),unitary_vector(1),unitary_vector(2),1/depth};
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
