#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//std includes
#include <iostream>

//wolf includes
#include "pinholeTools.h"


int main(int argc, char** argv)
{
    using namespace wolf;

    std::cout << std::endl << " ========= ProjectPoints test ===========" << std::endl << std::endl;

    cv::Point3f points3D;
    points3D.x = 2.0;
    points3D.y = 5.0;
    points3D.z = 6.0;
    std::vector<cv::Point3f> point_in_3D;
    point_in_3D.push_back(points3D);
    points3D.x = 4.0;
    points3D.y = 2.0;
    points3D.z = 1.0;
    point_in_3D.push_back(points3D);

    std::vector<float> rot_mat = {0,0,0};
    std::vector<float> trans_mat = {1,1,1};

    cv::Mat cam_mat(3,3,CV_32F);
    cam_mat.row(0).col(0).setTo(1);
    cam_mat.row(0).col(1).setTo(0);
    cam_mat.row(0).col(2).setTo(2);
    cam_mat.row(1).col(0).setTo(0);
    cam_mat.row(1).col(1).setTo(1);
    cam_mat.row(1).col(2).setTo(2);
    cam_mat.row(2).col(0).setTo(0);
    cam_mat.row(2).col(1).setTo(0);
    cam_mat.row(2).col(2).setTo(1);

    std::cout << "cam_mat[1,2]: " << cam_mat.row(1).col(0) << std::endl;

    std::vector<float> dist_coef = {0,0,0,0,0};
    std::vector<cv::Point2f> points2D;
    cv::projectPoints(point_in_3D,rot_mat,trans_mat,cam_mat,dist_coef,points2D);

    for (auto it : points2D)
    {
        std::cout << "points2D- X: " << it.x << "; Y: " << it.y << std::endl;
    }

    std::cout << std::endl << " ========= PinholeTools TESTING FUNCTIONS ===========" << std::endl << std::endl;

    //================================= distortPoint

    Eigen::Vector2s distortion;
    distortion[0] = 1;
    distortion[1] = 0.8;
    Eigen::Vector2s dis_point;
    dis_point[0] = 40;
    dis_point[1] = 40;

    Eigen::Vector2s distored_point;
    distored_point = pinhole::distortPoint(distortion,dis_point);

    std::cout << std::endl << "TEST distortPoint" << std::endl;
    std::cout << "x: " << distored_point[0] << "; y: " << distored_point[1] << std::endl;



    Eigen::Vector2s distored_point2;
    Eigen::Matrix2s distortion_jacobian;
    pinhole::distortPoint(distortion,dis_point,distored_point2,distortion_jacobian);

    std::cout << std::endl << "TEST distortPoint, jacobian" << std::endl;
    std::cout << "x: " << distored_point2[0] << "; y: " << distored_point2[1] << std::endl;
    std::cout << "Jacobian" << std::endl <<
                 distortion_jacobian.row(0).col(0) << " " << distortion_jacobian.row(0).col(1) << std::endl <<
                 distortion_jacobian.row(1).col(0) << " " << distortion_jacobian.row(1).col(1) << std::endl;

    //================================= undistortPoint

    Eigen::Vector2s correction;
    correction[0] = 1;
    correction[1] = 1.2;
    Eigen::Vector2s cor_point;
    cor_point[0] = 40;
    cor_point[1] = 40;

    Eigen::Vector2s corrected_point;
    corrected_point = pinhole::undistortPoint(correction,cor_point);

    std::cout << std::endl << "TEST undistortPoint" << std::endl;
    std::cout << "x: " << corrected_point[0] << "; y: " << corrected_point[1] << std::endl;



    Eigen::Vector2s corrected_point2;
    Eigen::Matrix2s corrected_jacobian;
    //distortion_jacobian(0, 0) = 1;
    pinhole::undistortPoint(correction,cor_point,corrected_point2,corrected_jacobian);

    std::cout << std::endl << "TEST undistortPoint, jacobian" << std::endl;
    std::cout << "x: " << corrected_point2[0] << "; y: " << corrected_point2[1] << std::endl;
    std::cout << "Jacobian" << std::endl <<
                 corrected_jacobian.row(0).col(0) << " " << corrected_jacobian.row(0).col(1) << std::endl <<
                 corrected_jacobian.row(1).col(0) << " " << corrected_jacobian.row(1).col(1) << std::endl;

    //================================= projectPoint Complete

    Eigen::Vector2s distortion_test;
    distortion_test[0] = 1;
    distortion_test[1] = 0.8;
    Eigen::Vector4f k_test;
    k_test[0] = 0.5;
    k_test[1] = 0.4;
    k_test[2] = 1.2;
    k_test[3] = 1.2;
    Eigen::Vector3s point3D_test;
    point3D_test[0] = 3;
    point3D_test[1] = 3;
    point3D_test[2] = 3;

    Eigen::Vector2s point2D_test;
    point2D_test = pinhole::projectPoint(k_test,distortion_test,point3D_test);

    std::cout << std::endl << "TEST projectPoint Complete" << std::endl;
    std::cout << "x: " << point2D_test[0] << "; y: " << point2D_test[1] << std::endl;


    Eigen::Vector2s point2D_test2;
    Scalar distance_test;
    pinhole::projectPoint(k_test,distortion_test,point3D_test,point2D_test2,distance_test);

    std::cout << std::endl << "TEST projectPoint Complete, distance" << std::endl;
    std::cout << "x: " << point2D_test2[0] << "; y: " << point2D_test2[1] << "; dist: " << distance_test << std::endl;


    Eigen::Vector2s point2D_test3;
    Eigen::MatrixXs jacobian_test(2,3);
    pinhole::projectPoint(k_test,distortion_test,point3D_test,point2D_test3,jacobian_test);

    std::cout << std::endl << "TEST projectPoint Complete, jacobian" << std::endl;
    std::cout << "x: " << point2D_test3[0] << "; y: " << point2D_test3[1] << std::endl;
    std::cout << "Jacobian" << std::endl <<
                 jacobian_test.row(0).col(0) << " " << jacobian_test.row(0).col(1) << " " << jacobian_test.row(0).col(2) << std::endl <<
                 jacobian_test.row(1).col(0) << " " << jacobian_test.row(1).col(1) << " " << jacobian_test.row(1).col(2) << std::endl;


    Eigen::Vector2s point2D_test4;
    Eigen::MatrixXs jacobian_test2(2,3);
    Scalar distance_test2;
    pinhole::projectPoint(k_test,distortion_test,point3D_test,point2D_test4,distance_test2,jacobian_test2);

    std::cout << std::endl << "TEST projectPoint Complete, distance and jacobian" << std::endl;
    std::cout << "x: " << point2D_test4[0] << "; y: " << point2D_test4[1] << "; dist: " << distance_test2 << std::endl;
    std::cout << "Jacobian" << std::endl <<
                 jacobian_test2.row(0).col(0) << " " << jacobian_test2.row(0).col(1) << " " << jacobian_test2.row(0).col(2) << std::endl <<
                 jacobian_test2.row(1).col(0) << " " << jacobian_test2.row(1).col(1) << " " << jacobian_test2.row(1).col(2) << std::endl;

    //================================= backprojectPoint Complete


    Eigen::Vector2s correction_test;
    correction_test[0] = 1;
    correction_test[1] = 0.8;
    Eigen::Vector2s point2D_backproj;
    point2D_backproj[0] = 3;
    point2D_backproj[1] = 3;
    Scalar depth2 = 3;

    Eigen::Vector3s point3D_backproj;
    point3D_backproj = pinhole::backprojectPoint(k_test,correction_test,point2D_backproj,depth2);

    std::cout << std::endl << "TEST backprojectPoint Complete" << std::endl;
    std::cout << "x: " << point3D_backproj[0] << "; y: " << point3D_backproj[1] << "; z: " << point3D_backproj[2] << std::endl;


    Eigen::Vector3s point3D_backproj2;
    Eigen::MatrixXs jacobian_backproj(3,2);
    Eigen::Vector3s depth_jacobian;
    pinhole::backProjectPoint(k_test,correction_test,point2D_backproj,depth2,point3D_backproj2,jacobian_backproj,depth_jacobian);

    std::cout << std::endl << "TEST backprojectPoint Complete, jacobian and depth jacobian" << std::endl;
    std::cout << "x: " << point3D_backproj2[0] << "; y: " << point3D_backproj2[1] << "; z: " << point3D_backproj2[2] << std::endl;
    std::cout << "Jacobian" << std::endl <<
                 jacobian_backproj.row(0).col(0) << " " << jacobian_backproj.row(0).col(1) << std::endl <<
                 jacobian_backproj.row(1).col(0) << " " << jacobian_backproj.row(1).col(1) << std::endl <<
                 jacobian_backproj.row(2).col(0) << " " << jacobian_backproj.row(2).col(1) << std::endl;
    std::cout << "Jacobian - depth" << std::endl <<
                 depth_jacobian[0] << " " << depth_jacobian[1] << " " << depth_jacobian[2] << " " << std::endl;


    std::cout << std::endl << " ========= PinholeTools DUALITY TEST ===========" << std::endl << std::endl;

    //================================= projectPoint and backprojectPoint to/from NormalizedPlane

    Eigen::Vector3s project_point_normalized_test;
    project_point_normalized_test[0] = 3;
    project_point_normalized_test[1] = 3;
    project_point_normalized_test[2] = 3;
    Eigen::Vector2s project_point_normalized_output;
    Eigen::Vector2s project_point_normalized_output2;
    Scalar project_point_normalized_dist;

    Scalar backproject_point_normalized_depth = 3;
    Eigen::Vector3s backproject_point_normalized_output;


    project_point_normalized_output = pinhole::projectPointToNormalizedPlane(project_point_normalized_test);
    pinhole::projectPointToNormalizedPlane(project_point_normalized_test,project_point_normalized_output2,project_point_normalized_dist);

    backproject_point_normalized_output =
            pinhole::backprojectPointFromNormalizedPlane(project_point_normalized_output,backproject_point_normalized_depth);


    std::cout << "TEST project and backproject PointToNormalizedPlane" << std::endl;
    std::cout << std:: endl << "Original" << std::endl;
    std::cout << "x: " << project_point_normalized_test[0] << "; y: " << project_point_normalized_test[1]
              << "; z: " << project_point_normalized_test[2] << std::endl;
    std::cout << std:: endl << "Project" << std::endl;
    std::cout << "x: " << project_point_normalized_output[0] << "; y: " << project_point_normalized_output[1]
              << "; rows: " << project_point_normalized_output.rows() << "; cols: " << project_point_normalized_output.cols()
              << std::endl;
    std::cout << std:: endl << "Alternate project" << std::endl;
    std::cout << "x: " << project_point_normalized_output[0] << "; y: " << project_point_normalized_output[1]
              << "; rows: " << project_point_normalized_output.rows() << "; cols: " << project_point_normalized_output.cols()
              << "; distance: " << project_point_normalized_dist << std::endl;
    std::cout << std:: endl << "Backproject" << std::endl;
    std::cout << "x: " << backproject_point_normalized_output[0] << "; y: " << backproject_point_normalized_output[1]
              << "; z: " << backproject_point_normalized_output[2] << "; depth: " << backproject_point_normalized_depth << std::endl;


    //================================= projectPoint and backprojectPoint to/from NormalizedPlane WITH JACOBIANS

    Eigen::Vector3s pp_normalized_test;
    pp_normalized_test[0] = 3;
    pp_normalized_test[1] = 3;
    pp_normalized_test[2] = 3;
    Eigen::Vector2s pp_normalized_output;
    Eigen::Vector2s pp_normalized_output2;
    Eigen::Matrix3s pp_normalized_jacobian;
    Eigen::Matrix3s pp_normalized_jacobian2;
    Scalar pp_normalized_distance;

    Scalar bpp_normalized_depth = 3;
    Eigen::Vector3s bpp_normalized_output;
    Eigen::Matrix3s bpp_normalized_jacobian;
    Eigen::Vector3s bpp_normalized_jacobian_depth;


    pinhole::projectPointToNormalizedPlane(pp_normalized_test,pp_normalized_output,pp_normalized_jacobian);
    pinhole::projectPointToNormalizedPlane(pp_normalized_test,pp_normalized_output2,pp_normalized_distance,pp_normalized_jacobian2);

    pinhole::backprojectPointFromNormalizedPlane(pp_normalized_output,bpp_normalized_depth,
                                                 bpp_normalized_output,bpp_normalized_jacobian,bpp_normalized_jacobian_depth);


    std::cout << std::endl << std::endl << "TEST project and backproject PointToNormalizedPlane with JACOBIAN" << std::endl;

    std::cout << std:: endl << "Original" << std::endl;
    std::cout << "x: " << pp_normalized_test[0] << "; y: " << pp_normalized_test[1] << "; z: " << pp_normalized_test[2] << std::endl;
    std::cout << std:: endl << "Project" << std::endl;
    std::cout << "x: " << pp_normalized_output[0] << "; y: " << pp_normalized_output[1] << "; rows: " << pp_normalized_output.rows()
              << "; cols: " << pp_normalized_output.cols() << std::endl;
    std::cout << "--> Jacobian" << std::endl <<
                 pp_normalized_jacobian.row(0).col(0) << " " << pp_normalized_jacobian.row(0).col(1) << " " << pp_normalized_jacobian.row(0).col(2) << " " << std::endl <<
                 pp_normalized_jacobian.row(1).col(0) << " " << pp_normalized_jacobian.row(1).col(1) << " " << pp_normalized_jacobian.row(1).col(2) << " " << std::endl <<
                 pp_normalized_jacobian.row(2).col(0) << " " << pp_normalized_jacobian.row(2).col(1) << " " << pp_normalized_jacobian.row(2).col(2) << " " << std::endl;

    std::cout << std:: endl << "Alternate project" << std::endl;
    std::cout << "x: " << pp_normalized_output2[0] << "; y: " << pp_normalized_output2[1] << "; rows: "
              << pp_normalized_output2.rows() << "; cols: " << pp_normalized_output2.cols() << "; distance: "
              << pp_normalized_distance << std::endl;
    std::cout << "Jacobian" << std::endl <<
                 pp_normalized_jacobian2.row(0).col(0) << " " << pp_normalized_jacobian2.row(0).col(1) << " " << pp_normalized_jacobian2.row(0).col(2) << " " << std::endl <<
                 pp_normalized_jacobian2.row(1).col(0) << " " << pp_normalized_jacobian2.row(1).col(1) << " " << pp_normalized_jacobian2.row(1).col(2) << " " << std::endl <<
                 pp_normalized_jacobian2.row(2).col(0) << " " << pp_normalized_jacobian2.row(2).col(1) << " " << pp_normalized_jacobian2.row(2).col(2) << " " << std::endl;

    std::cout << std:: endl << "Backproject" << std::endl;
    std::cout << "x: " << bpp_normalized_output[0] << "; y: " << bpp_normalized_output[1] << "; z: " << bpp_normalized_output[2]
              << "; depth: " << bpp_normalized_depth << std::endl;
    std::cout << "--> Jacobian" << std::endl <<
                 bpp_normalized_jacobian.row(0).col(0) << " " << bpp_normalized_jacobian.row(0).col(1) << " " << bpp_normalized_jacobian.row(0).col(2) << " " << std::endl <<
                 bpp_normalized_jacobian.row(1).col(0) << " " << bpp_normalized_jacobian.row(1).col(1) << " " << bpp_normalized_jacobian.row(1).col(2) << " " << std::endl <<
                 bpp_normalized_jacobian.row(2).col(0) << " " << bpp_normalized_jacobian.row(2).col(1) << " " << bpp_normalized_jacobian.row(2).col(2) << " " << std::endl;
    std::cout << "--> Jacobian - depth" << std::endl <<
                 bpp_normalized_jacobian_depth[0] << " " << bpp_normalized_jacobian_depth[1] << " " << bpp_normalized_jacobian_depth[2] << " " << std::endl;


    Eigen::Matrix3s test_jacobian;
    test_jacobian =  pp_normalized_jacobian * bpp_normalized_jacobian;

    std::cout << std::endl << "Jacobian Testing" << std::endl <<
                 test_jacobian.row(0).col(0) << " " << test_jacobian.row(0).col(1) << " " << test_jacobian.row(0).col(2) << " " << std::endl <<
                 test_jacobian.row(1).col(0) << " " << test_jacobian.row(1).col(1) << " " << test_jacobian.row(1).col(2) << " " << std::endl <<
                 test_jacobian.row(2).col(0) << " " << test_jacobian.row(2).col(1) << " " << test_jacobian.row(2).col(2) << " " << std::endl;


    //================================= IsInRoi / IsInImage

    Eigen::Vector2s pix;
    pix[0] = 40; // x
    pix[1] = 40; // y

    int roi_x = 30;
    int roi_y = 30;
    int roi_width = 20;
    int roi_height = 20;

    int image_width = 640;
    int image_height = 480;

    bool is_in_roi;
    bool is_in_image;
    is_in_roi = pinhole::isInRoi(pix,roi_x,roi_y,roi_width,roi_height);
    is_in_image = pinhole::isInImage(pix,image_width,image_height);

    std::cout << std::endl << std::endl << "TEST isInRoi/isInImage" << std::endl;
    std::cout << std::endl << "Pixel " << std::endl;
    std::cout << "x: " << pix[0] << "; y: " << pix[1] << std::endl;
    std::cout << std::endl << "ROI " << std::endl;
    std::cout << "x: " << roi_x << "; y: " << roi_y << "; width: " << roi_width << "; height: " << roi_height << std::endl;
    std::cout << "is_in_roi: " << is_in_roi << std::endl;
    std::cout << std::endl << "Image " << std::endl;
    std::cout << "width: " << image_width << "; height: " << image_height << std::endl;
    std::cout << "is_in_image: " << is_in_image << std::endl;



    //================================= computeCorrectionModel

    Eigen::Vector2s distortion2;
    distortion2[0] = -0.301701;
    distortion2[1] = 0.0963189;
    Eigen::Vector4f k_test2;
    //k = [u0, v0, au, av]
    k_test2[0] = 516.686; //u0
    k_test2[1] = 355.129; //v0
    k_test2[2] = 991.852; //au
    k_test2[3] = 995.269; //av

    Eigen::Vector2s correction_test2;
    pinhole::computeCorrectionModel(k_test2,distortion2,correction_test2);

    std::cout << std::endl << std::endl << "TEST computeCorrectionModel" << std::endl;
    std::cout << "d1: " << distortion2[0] << "; d2: " << distortion2[1] << std::endl;
    std::cout << "u0: " << k_test2[0] << "; v0: " << k_test2[1] << "; au: " << k_test2[2] << "; av: " << k_test2[3] << std::endl;
    std::cout << "c1: " << correction_test2[0] << "; c2: " << correction_test2[1] << std::endl;



    //================================= distortPoint


    Eigen::Vector2s distorting_point;
    distorting_point[0] = 2;
    distorting_point[1] = 2;

    Eigen::Vector2s distored_point3;
    distored_point3 = pinhole::distortPoint(distortion2,distorting_point);

    std::cout << std::endl << "TEST distortPoint" << std::endl;
    std::cout << "x: " << distorting_point[0] << "; y: " << distorting_point[1] << std::endl;
    std::cout << "x: " << distored_point3[0] << "; y: " << distored_point3[1] << std::endl;

    Eigen::Vector2s corrected_point4;
    corrected_point4 = pinhole::undistortPoint(correction_test2,distored_point3);
    std::cout << "x: " << corrected_point4[0] << "; y: " << corrected_point4[1] << std::endl;



    //================================= PixelizePoint

    Eigen::Vector2s pixelize_ud;
    pixelize_ud[0] = 45;
    pixelize_ud[1] = 28;

    Eigen::Vector2s pixelize_output3;
    pixelize_output3 = pinhole::pixellizePoint(k_test2,pixelize_ud);

    std::cout << std::endl << "TEST pixelizePoint; Eigen::Vector2s" << std::endl;
    std::cout << std::endl << "Original" << std::endl;
    std::cout << "x: " << pixelize_ud[0] << "; y: " << pixelize_ud[1] << std::endl;
    std::cout << std::endl << "Pixelized" << std::endl;
    std::cout << "x: " << pixelize_output3[0] << "; y: " << pixelize_output3[1] << std::endl;

    Eigen::Vector2s depixelize_output3;
    depixelize_output3 = pinhole::depixellizePoint(k_test2,pixelize_output3);
    std::cout << std::endl << "Depixelized" << std::endl;
    std::cout << "x: " << depixelize_output3[0] << "; y: " << depixelize_output3[1] << std::endl;


    ////

    Eigen::Vector2s pixelize_output4;
    Eigen::Matrix2s pixelize_jacobian2;
    pinhole::pixellizePoint(k_test2,pixelize_ud,pixelize_output4,pixelize_jacobian2);

    std::cout << std::endl << "TEST pixelizePoint; Jacobians" << std::endl;
    std::cout << std::endl << "Original" << std::endl;
    std::cout << "x: " << pixelize_ud[0] << "; y: " << pixelize_ud[1] << std::endl;
    std::cout << std::endl << "Pixelized" << std::endl;
    std::cout << "x: " << pixelize_output4[0] << "; y: " << pixelize_output4[1] << std::endl;
    std::cout << "--> Jacobian" << std::endl <<
                 pixelize_jacobian2.row(0).col(0) << " " << pixelize_jacobian2.row(0).col(1) << std::endl <<
                 pixelize_jacobian2.row(1).col(0) << " " << pixelize_jacobian2.row(1).col(1) << std::endl;


    Eigen::Vector2s depixelize_output4;
    Eigen::Matrix2s depixelize_jacobian2;
    pinhole::depixellizePoint(k_test2,pixelize_output4,depixelize_output4,depixelize_jacobian2);

    std::cout << std::endl << "Depixelized" << std::endl;
    std::cout << "x: " << depixelize_output4[0] << "; y: " << depixelize_output4[1] << std::endl;
    std::cout << "--> Jacobian" << std::endl <<
                 depixelize_jacobian2.row(0).col(0) << " " << depixelize_jacobian2.row(0).col(1) << std::endl <<
                 depixelize_jacobian2.row(1).col(0) << " " << depixelize_jacobian2.row(1).col(1) << std::endl;

    Eigen::Matrix2s test_jacobian_pix;
    test_jacobian_pix =  pixelize_jacobian2 * depixelize_jacobian2;

    std::cout << std::endl << "Jacobian Testing" << std::endl <<
                 test_jacobian_pix.row(0).col(0) << " " << test_jacobian_pix.row(0).col(1) << std::endl <<
                 test_jacobian_pix.row(1).col(0) << " " << test_jacobian_pix.row(1).col(1) << std::endl;
}







