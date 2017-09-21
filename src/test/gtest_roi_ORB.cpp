/**
 * \file gtest_roi_ORB.cpp
 *
 *  Created on: Nov 28, 2016
 *      \author: jsola
 */


#include "utils_gtest.h"

#include "wolf.h"
#include "logging.h"

// vision utils includes
#include <vision_utils.h>

// REMOVE
//opencv includes
//#if defined (HAVE_OPENCV3)
//#include <opencv2/features2d.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/core.hpp>
//#include <opencv2/imgproc.hpp>
//#else
//#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/core/core.hpp>
//#endif

// std include
#include <vector>
#include <map>

using namespace wolf;
using namespace Eigen;
//using namespace cv;

TEST(RoiORB, LoadImageFromFile)
{
    cv::Mat image;
    std::string filename, wolf_root;
    wolf_root = _WOLF_ROOT_DIR;
    filename = wolf_root + "/src/examples/Test_ORB.png";
    image = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    ASSERT_TRUE(image.data)<< "Failed to load image " << filename << std::endl;
}

// Test_ORB.png image file has the following interest points that should be detected:
// 0  [ 65.6 , 100.6 ]
// 1  [ 164 , 100.6 ]
// 2  [ 266 , 100.6 ]
// 3  [ 365.2 , 100.6 ]
// 4  [ 467.04 , 101.32 ]
// 5  [ 565.6 , 100.6 ]
// 6  [ 71.6 , 237.8 ]
// 7  [ 164.8 , 237.8 ]
// 8  [ 251.36 , 239.24 ]
// 9  [ 330.2 , 237.8 ]
// 10 [ 349 , 270 ]     // point #10 is out of the ROI scanned area and should not be detected
// 11 [ 584 , 237.8 ]
// 12 [ 467.2 , 455.4 ]
// 13 [ 566 , 455.4 ]
std::vector<cv::Point2f> points_to_check({
    cv::Point2f(  65.6 , 100.6 ),
    cv::Point2f( 164   , 100.6 ),
    cv::Point2f( 266   , 100.6 ),
    cv::Point2f( 365.2 , 100.6 ),
    cv::Point2f( 467   , 101.3 ),
    cv::Point2f( 565.6 , 100.6 ),
    cv::Point2f(  71.6 , 237.8 ),
    cv::Point2f( 164.8 , 237.8 ),
    cv::Point2f( 250   , 239.2 ),
    cv::Point2f( 330.2 , 237.8 ),
    cv::Point2f( 349   , 270   ),
    cv::Point2f( 584   , 237.8 ),
    cv::Point2f( 467.2 , 455.4 ),
    cv::Point2f( 566   , 455.4 )
});

/* \brief Chack that p is one of the points in the list p_vec, and return the index to the matching point
 *
 * @param p         point to test
 * @param p_vec     vector of points to be checked against
 * @param pixel_tol maximum distance in pixels to declare a match
 * @return          if found, index of the matching point in p_vec; -1 if not found
 */
int existsIn(const cv::Point2f& p, const std::vector<cv::Point2f> p_vec, const Scalar& pixel_tol)
{
    Scalar pixel_tol_squared = pixel_tol*pixel_tol;
    for (int i = 0; i < p_vec.size(); i++)
    {
        Scalar dx = p.x - p_vec[i].x;
        Scalar dy = p.y - p_vec[i].y;
        if ( dx*dx + dy*dy < pixel_tol_squared) // match based on Euclidean distance
            return i;
    }
    return -1; // -1 marks 'not found'
}

TEST(RoiORB, RoiBounds)
{
#ifdef _WOLF_DEBUG
    bool debug = true;
#else
    bool debug = false;
#endif

    cv::Mat image;
    std::string filename, wolf_root;
    wolf_root = _WOLF_ROOT_DIR;
    filename = wolf_root + "/src/examples/Test_ORB.png";
    image = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

    ASSERT_TRUE(image.data)<< "failed to load image " << filename << std::endl;

    WOLF_DEBUG( "succeded" );
    unsigned int img_width = image.cols;
    unsigned int img_height = image.rows;
    WOLF_DEBUG( "Image size: " , img_width , "x" , img_height );

    unsigned int nfeatures = 20;
    float scaleFactor = 1.2;
    unsigned int nlevels = 8;
    unsigned int edgeThreshold = 16;
    unsigned int firstLevel = 0;
    unsigned int WTA_K = 2;                  //# See: http://docs.opencv.org/trunk/db/d95/classcv_1_1ORB.html#a180ae17d3300cf2c619aa240d9b607e5
    unsigned int scoreType = 0;              //#enum { kBytes = 32, HARRIS_SCORE=0, FAST_SCORE=1 };
    unsigned int patchSize = 31;

//#if defined (HAVE_OPENCV3)
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create(nfeatures, //
                                                            scaleFactor, //
                                                            nlevels, //
                                                            edgeThreshold, //
                                                            firstLevel, //
                                                            WTA_K, //
                                                            scoreType, //
                                                            patchSize);
//REMOVE
//#else
//    std::shared_ptr<cv::ORB> detector = std::make_shared<cv::ORB>(nfeatures, //
//                                                                  scaleFactor, //
//                                                                  nlevels, //
//                                                                  edgeThreshold, //
//                                                                  firstLevel, //
//                                                                  WTA_K, //
//                                                                  scoreType, //
//                                                                  patchSize);
//#endif

    std::vector<cv::KeyPoint> target_keypoints;
    cv::KeyPointsFilter keypoint_filter;

    Size roi_x;
    Size roi_y;
    Size roi_width = 50;
    Size roi_heigth = 50;

    VectorXi roi_center_y(3); roi_center_y << 102 , 250 , 476;

    std::map<int, cv::Point2f> points_found;

    for (Size i = 0; i<roi_center_y.size() ; i++)
    {
        roi_y = roi_center_y(i) - roi_width/2;

        for(roi_x = 0; roi_x < img_width; roi_x += 5)
        {
            cv::Rect roi(roi_x, roi_y, roi_width, roi_heigth);
            cv::Rect roi_inflated = roi;

            roi_inflated.x = roi.x - edgeThreshold;
            roi_inflated.y = roi.y - edgeThreshold;
            roi_inflated.width = roi.width + 2*edgeThreshold;
            roi_inflated.height = roi.height + 2*edgeThreshold;

            if(roi_inflated.x < 0)
            {
                int diff_x = -roi_inflated.x;
                roi_inflated.x = 0;
                roi_inflated.width = roi_inflated.width - diff_x;
            }
            if(roi_inflated.y < 0)
            {
                int diff_y = -roi_inflated.y;
                roi_inflated.y = 0;
                roi_inflated.height = roi_inflated.height - diff_y;
            }
            if((unsigned int)(roi_inflated.x + roi_inflated.width) > img_width)
            {
                int diff_width = img_width - (roi_inflated.x + roi_inflated.width);
                roi_inflated.width = roi_inflated.width + diff_width;
            }
            if((unsigned int)(roi_inflated.y + roi_inflated.height) > img_height)
            {
                int diff_height = img_height - (roi_inflated.y + roi_inflated.height);
                roi_inflated.height = roi_inflated.height+diff_height;
            }

            cv::Mat image_roi = image(roi_inflated);
            target_keypoints.clear();
            detector->detect(image_roi, target_keypoints);

            if (!target_keypoints.empty())
            {
                if (debug) std::cout << "Keypoints detected: " << target_keypoints.size() << "  at: ";

                for (Size i = 0; i < target_keypoints.size(); i++)
                {
                    target_keypoints[i].pt.x += roi_inflated.x;
                    target_keypoints[i].pt.y += roi_inflated.y;

                    if (debug) std::cout << "[ " << target_keypoints[i].pt.x << " , " << target_keypoints[i].pt.y << " ] ";
                }
                if (debug) std::cout << std::endl;

                keypoint_filter.retainBest(target_keypoints,1);

                cv::Point2f pnt = target_keypoints[0].pt;

                if (debug) std::cout << "          retained: " << target_keypoints.size()
                                         << "  at: "
                                         << "[ " << pnt.x << " , " << pnt.y << " ] "
                                         << std::endl;

                int j = existsIn(pnt, points_to_check, 2.0);

                ASSERT_GE(j, 0);

                // append the keypoint to the list of keypoints found
                if (j >= 0)
                    points_found[j] = pnt;
            }


#ifdef _WOLF_DEBUG
            cv::Mat image_graphics = image.clone();
            cv::drawKeypoints(image_graphics,target_keypoints,image_graphics);
            cv::rectangle(image_graphics, roi, cv::Scalar(255.0, 0.0, 255.0), 1, 8, 0);
            cv::rectangle(image_graphics, roi_inflated, cv::Scalar(255.0, 255.0, 0.0), 1, 8, 0);

            cv::imshow("test",image_graphics);
            cv::waitKey(1);
#endif
            }
    }

    // check that at least all keypoints in the list except #10 have been detected
    // (note: #10 is out of the ROI)
    std::vector<int> nn({0,1,2,3,4,5,6,7,8,9,11,12,13});
    for (int n : nn)
    {
        WOLF_DEBUG("Checking if we found point " , n , " ... " , (points_found.count(n) ? "YES": "NO"));
        ASSERT_TRUE(points_found.count(n));
    }

    WOLF_DEBUG("Checking if we found point " , 10 , " ... " , (points_found.count(10) ? "YES": "NO"));
    ASSERT_FALSE(points_found.count(10));

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
