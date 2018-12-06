/**
 * \file gtest_landmark_apriltag.cpp
 *
 *  Created on: Dec 6, 2018
 *      \author: jsola
 */


#include "utils_gtest.h"


#include "wolf.h"
#include "logging.h"

#include "landmark_apriltag.h"

using namespace Eigen;
using namespace wolf;
using std::static_pointer_cast;

class LandmarkApriltag_class : public testing::Test{
    public:
        virtual void SetUp()
        {
            wolf_root = _WOLF_ROOT_DIR;
            problem = Problem::create("PO 3D");
        }
    public:
        std::string wolf_root;
        ProblemPtr   problem;
};

//TEST(TestGroup, DummyTestExample)
//{
//    // TODO: Automatically generated TEST stub
//}

TEST_F(LandmarkApriltag_class, loadSaveMap)
{
    problem->loadMap(wolf_root + "/src/examples/map_apriltag_1.yaml");
    ASSERT_EQ(problem->getMapPtr()->getLandmarkList().size(), 4);

    problem->saveMap(wolf_root + "/src/examples/map_apriltag_save.yaml");

    problem->getMapPtr()->getLandmarkList().clear();
    ASSERT_EQ(problem->getMapPtr()->getLandmarkList().size(), 0);

    problem->loadMap(wolf_root + "/src/examples/map_apriltag_save.yaml");
    ASSERT_EQ(problem->getMapPtr()->getLandmarkList().size(), 4);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

