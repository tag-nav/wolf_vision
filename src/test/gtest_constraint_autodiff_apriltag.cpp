#include "utils_gtest.h"

#include "wolf.h"
#include "logging.h"

#include "constraints/constraint_autodiff_apriltag.h"

using namespace Eigen;
using namespace wolf;
using std::static_pointer_cast;

// Use the following in case you want to initialize tests with predefines variables or methods.
//class ConstraintAutodiffApriltag_class : public testing::Test{
//    public:
//        virtual void SetUp()
//        {
//        }
//};

TEST(ConstraintAutodiffApriltag, Constructor)
{
    std::cout << "\033[1;33m [WARN]:\033[0m gtest for ConstraintAutodiffApriltag Constructor is empty." << std::endl;
}

TEST(ConstraintAutodiffApriltag, Destructor)
{
    std::cout << "\033[1;33m [WARN]:\033[0m gtest for ConstraintAutodiffApriltag Destructor is empty." << std::endl;
}

//[Class methods]

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

