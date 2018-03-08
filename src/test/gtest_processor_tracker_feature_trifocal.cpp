#include "utils_gtest.h"

#include "wolf.h"
#include "logging.h"

#include "processors/processor_tracker_feature_trifocal.h"

using namespace Eigen;
using namespace wolf;
using std::static_pointer_cast;

// Use the following in case you want to initialize tests with predefines variables or methods.
//class ProcessorTrackerFeatureTrifocal_class : public testing::Test{
//    public:
//        virtual void SetUp()
//        {
//        }
//};

TEST(ProcessorTrackerFeatureTrifocal, Constructor)
{
  std::cout << "\033[1;33m [WARN]:\033[0m gtest for ProcessorTrackerFeatureTrifocal Constructor is empty." << std::endl;
}

TEST(ProcessorTrackerFeatureTrifocal, Destructor)
{
  std::cout << "\033[1;33m [WARN]:\033[0m gtest for ProcessorTrackerFeatureTrifocal Destructor is empty." << std::endl;
}

//[Class methods]
TEST(ProcessorTrackerFeatureTrifocal, trackFeatures)
{
  std::cout << "033[1;33m [WARN]:033[0m gtest for ProcessorTrackerFeatureTrifocal trackFeatures is empty." << std::endl;
}

TEST(ProcessorTrackerFeatureTrifocal, correctFeatureDrift)
{
  std::cout << "033[1;33m [WARN]:033[0m gtest for ProcessorTrackerFeatureTrifocal correctFeatureDrift is empty." << std::endl;
}

TEST(ProcessorTrackerFeatureTrifocal, voteForKeyFrame)
{
  std::cout << "033[1;33m [WARN]:033[0m gtest for ProcessorTrackerFeatureTrifocal voteForKeyFrame is empty." << std::endl;
}

TEST(ProcessorTrackerFeatureTrifocal, detectNewFeatures)
{
  std::cout << "033[1;33m [WARN]:033[0m gtest for ProcessorTrackerFeatureTrifocal detectNewFeatures is empty." << std::endl;
}

TEST(ProcessorTrackerFeatureTrifocal, createConstraint)
{
  std::cout << "033[1;33m [WARN]:033[0m gtest for ProcessorTrackerFeatureTrifocal createConstraint is empty." << std::endl;
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

