#include "landmark_AHP.h"
#include "constraint_AHP.h"
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
    ProblemPtr wolf_problem_ptr_ = std::make_shared<Problem>(FRM_PO_3D);
    wolf_problem_ptr_->setup();

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

    SensorBasePtr sensor_ptr = wolf_problem_ptr_->installSensor("CAMERA", "PinHole", extr, wolf_path + "/src/examples/camera_params.yaml");
    std::shared_ptr<SensorCamera> camera_ptr_ = std::static_pointer_cast<SensorCamera>(sensor_ptr);

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

    FrameBasePtr last_frame = std::make_shared<FrameBase>(t,std::make_shared<StateBlock>(frame_val.head(3)), std::make_shared<StateQuaternion>(frame_val.tail(4)));
    std::cout << "Last frame" << std::endl;
    wolf_problem_ptr_->getTrajectoryPtr()->addFrame(last_frame);

    // Capture
    std::shared_ptr<CaptureImage> image_ptr;
    t.setToNow();
    cv::Mat frame; //puede que necesite una imagen

    image_ptr = std::make_shared< CaptureImage>(t, camera_ptr_, frame);
    wolf_problem_ptr_->getLastFramePtr()->addCapture(image_ptr);


    // create the feature
    cv::KeyPoint kp; kp.pt = {40,40};
    cv::Mat desc;

    std::shared_ptr<FeaturePointImage> feat_point_image_ptr = std::make_shared<FeaturePointImage>(kp, desc, Eigen::Matrix2s::Identity());
    image_ptr->addFeature(feat_point_image_ptr);

    FrameBasePtr anchor_frame = std::make_shared< FrameBase>(t,std::make_shared<StateBlock>(frame_val.head(3)), std::make_shared<StateQuaternion>(frame_val.tail(4)));
    //FrameBasePtr anchor_frame = wolf_problem_ptr_->getTrajectoryPtr()->getLastFramePtr();


    // create the landmark
    Eigen::Vector2s point2D;
    point2D[0] = feat_point_image_ptr->getKeypoint().pt.x;
    point2D[1] = feat_point_image_ptr->getKeypoint().pt.y;
    std::cout << "point2D: " << point2D.transpose() << std::endl;


    Scalar distance = 2; // arbitrary value
    Eigen::Vector4s vec_homogeneous;

    point2D = pinhole::depixellizePoint(image_ptr->getSensorPtr()->getIntrinsicPtr()->getVector(),point2D);
    std::cout << "point2D depixellized: " << point2D.transpose() << std::endl;
    point2D = pinhole::undistortPoint((std::static_pointer_cast<SensorCamera>(image_ptr->getSensorPtr()))->getCorrectionVector(),point2D);
    std::cout << "point2D undistorted: " << point2D.transpose() << std::endl;

    Eigen::Vector3s point3D;
    point3D.head(2) = point2D;
    point3D(2) = 1;

    point3D.normalize();
    std::cout << "point3D normalized: " << point3D.transpose() << std::endl;

    vec_homogeneous = {point3D(0),point3D(1),point3D(2),1/distance};
    std::cout << "vec_homogeneous: " << vec_homogeneous.transpose() << std::endl;

    std::shared_ptr<LandmarkAHP> landmark = std::make_shared<LandmarkAHP>(vec_homogeneous, anchor_frame, image_ptr->getSensorPtr(), feat_point_image_ptr->getDescriptor());

    std::cout << "Landmark AHP created" << std::endl;



    // Create the constraint
    ConstraintAHP::Ptr constraint_ptr = std::make_shared<ConstraintAHP>(feat_point_image_ptr, last_frame,std::static_pointer_cast<LandmarkAHP>(landmark));

    feat_point_image_ptr->addConstraint(constraint_ptr);
    std::cout << "Constraint AHP created" << std::endl;



    Eigen::Vector2s residuals;
    //    Eigen::Vector3s current_frame_p = last_frame->getPPtr()->getVector();
    //    Eigen::Vector4s current_frame_o = last_frame->getOPtr()->getVector();
    //    Eigen::Vector3s anchor_frame_p = landmark->getAnchorFrame()->getPPtr()->getVector();
    //    Eigen::Vector4s anchor_frame_o = landmark->getAnchorFrame()->getOPtr()->getVector();
    //    Eigen::Vector4s landmark_ = landmark->getPPtr()->getVector();
    //
    //    ( * constraint_ptr ) (current_frame_p.data(), current_frame_o.data(),
    //            anchor_frame_p.data(), anchor_frame_o.data(),
    //            landmark_.data(), residuals.data());
    // current frame p; current frame o; anchor frame p; anchor frame o; homogeneous vector landmark, residual


    std::cout << "Residual computed" << std::endl;
    std::cout << "Residual = " << residuals[0] << "   " << residuals[1] << std::endl;

    return 0;

}
