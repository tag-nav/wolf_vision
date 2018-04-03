#include "utils_gtest.h"

#include "wolf.h"
#include "logging.h"

#include "header_file"

using namespace Eigen;
using namespace wolf;
using std::static_pointer_cast;

// Use the following in case you want to initialize tests with predefines variables or methods.
//class class_name_class : public testing::Test{
//    public:
//        virtual void SetUp()
//        {
//        }
//};

TEST(class_name, Constructor)
{
    std::cout << "\033[1;33m [WARN]:\033[0m gtest for class_name Constructor is empty." << std::endl;
}

TEST(class_name, Destructor)
{
    std::cout << "\033[1;33m [WARN]:\033[0m gtest for class_name Destructor is empty." << std::endl;
}

//[Class methods]

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

