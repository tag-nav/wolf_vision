#include "utils_gtest.h"

#include "wolf.h"
#include "logging.h"

#include "processors/processor_tracker_landmark_apriltag.h"

using namespace Eigen;
using namespace wolf;
using std::static_pointer_cast;

// Use the following in case you want to initialize tests with predefines variables or methods.
//class ProcessorTrackerLandmarkApriltag_class : public testing::Test{
//    public:
//        virtual void SetUp()
//        {
//        }
//};

TEST(ProcessorTrackerLandmarkApriltag, Constructor)
{
    std::cout << "\033[1;33m [WARN]:\033[0m gtest for ProcessorTrackerLandmarkApriltag Constructor is empty." << std::endl;
}

TEST(ProcessorTrackerLandmarkApriltag, Destructor)
{
    std::cout << "\033[1;33m [WARN]:\033[0m gtest for ProcessorTrackerLandmarkApriltag Destructor is empty." << std::endl;
}

//[Class methods]
TEST(ProcessorTrackerLandmarkApriltag, _feature_landmark_correspondences))
{
  std::cout << "033[1;33m [WARN]:033[0m gtest for ProcessorTrackerLandmarkApriltag _feature_landmark_correspondences) is empty." << std::endl;
}

TEST(ProcessorTrackerLandmarkApriltag, voteForKeyFrame)
{
  std::cout << "033[1;33m [WARN]:033[0m gtest for ProcessorTrackerLandmarkApriltag voteForKeyFrame is empty." << std::endl;
}

TEST(ProcessorTrackerLandmarkApriltag, detectNewFeatures)
{
  std::cout << "033[1;33m [WARN]:033[0m gtest for ProcessorTrackerLandmarkApriltag detectNewFeatures is empty." << std::endl;
}

TEST(ProcessorTrackerLandmarkApriltag, createLandmark)
{
  std::cout << "033[1;33m [WARN]:033[0m gtest for ProcessorTrackerLandmarkApriltag createLandmark is empty." << std::endl;
}

TEST(ProcessorTrackerLandmarkApriltag, createConstraint)
{
  std::cout << "033[1;33m [WARN]:033[0m gtest for ProcessorTrackerLandmarkApriltag createConstraint is empty." << std::endl;
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

