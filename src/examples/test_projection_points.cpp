#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//std includes
#include <iostream>


int main(int argc, char** argv)
{
    std::cout << std::endl << " ========= ProjectPoints test ===========" << std::endl << std::endl;

    std::cout << "0" << std::endl;
    //std::vector<float> points3D = {2,3,5};
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

    std::cout << "1" << std::endl;
    std::vector<float> rot_mat = {0,0,0};
    std::vector<float> trans_mat = {1,1,1};

    std::cout << "2" << std::endl;
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

    std::cout << "3" << std::endl;
    std::vector<float> dist_coef = {0,0,0,0,0};
    //std::vector<float> points2D;
    std::vector<cv::Point2f> points2D;

    std::cout << "4" << std::endl;
    cv::projectPoints(point_in_3D,rot_mat,trans_mat,cam_mat,dist_coef,points2D);
    std::cout << "5" << std::endl;

    for (auto it : points2D)
    {
        std::cout << "points2D- X: " << it.x << "; Y: " << it.y << std::endl;
    }

    std::cout << std::endl << " ========= Brisk test ===========" << std::endl << std::endl;

    const char * Pim="/home/jtarraso/Escritorio/Test Brisk 2 - 40 40 - 1 punto.jpg";   // object

    cv::Mat Gray =cv::imread(Pim);
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    int Threshl=30;
    int Octaves=0; //(pyramid layer) from which the keypoint has been extracted
    float PatternScales=1.0f;

    cv::BRISK BRISKD(Threshl,Octaves,PatternScales);//initialize algoritm
    BRISKD.create("Feature2D.BRISK");

    BRISKD.detect(Gray, keypoints);


    for(unsigned int i = 0; i < keypoints.size();i++)
    {
        cv::circle(Gray, keypoints[i].pt, 2, cv::Scalar(80.0, 80.0, 254.0), -1, 3, 0);
    }

    cv::KeyPoint kp= keypoints[0];
    //std::cout << "keypoint." << kp.pt << std::endl;

    kp.pt.x = 20;//619;
    kp.pt.y = 339;

    std::cout << "keypoint." << kp.pt << std::endl;
    std::vector<cv::KeyPoint> v_kp;
    v_kp.push_back(kp);

//    kp.pt.x = 50;
//    kp.pt.y = 20;
//    v_kp.push_back(kp);

    BRISKD.compute(Gray, v_kp,descriptors);

    std::cout << "descriptor: " << descriptors.rows << "," << descriptors.cols << std::endl;
    std::cout << "descriptor: " << descriptors.row(0) << std::endl;

    cv::circle(Gray, kp.pt, 2, cv::Scalar(250.0, 90.0, 4.0), -1, 3, 0);

    cv::imshow("Last", Gray);
    cv::waitKey(0);
}







