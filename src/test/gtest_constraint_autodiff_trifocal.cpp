#include "utils_gtest.h"

#include "wolf.h"
#include "logging.h"

#include "constraints/constraint_autodiff_trifocal.h"

using namespace Eigen;
using namespace wolf;
using std::static_pointer_cast;

// Use the following in case you want to initialize tests with predefines variables or methods.
//class ConstraintAutodiffTrifocal_class : public testing::Test{
//    public:
//        virtual void SetUp()
//        {
//        }
//};

TEST(ConstraintAutodiffTrifocal, Constructor)
{
  std::cout << "\033[1;33m [WARN]:\033[0m gtest for ConstraintAutodiffTrifocal Constructor is empty." << std::endl;
}

TEST(ConstraintAutodiffTrifocal, Destructor)
{
  std::cout << "\033[1;33m [WARN]:\033[0m gtest for ConstraintAutodiffTrifocal Destructor is empty." << std::endl;
}

//[Class methods]

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

