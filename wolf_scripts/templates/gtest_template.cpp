#include "utils_gtest.h"

#include "wolf.h"
#include "logging.h"

#include "header_file"

using namespace Eigen;
using namespace wolf;
using std::static_pointer_cast;

// Use the following in case you want to initialize tests with predefines variables or methods.
class class_name : public testing::Test{
    public:
        virtual void SetUp()
        {
        }
};

TEST_F(class_name, Constructor)
{
}

TEST_F(class_name, Destructor)
{
}

//[Class methods]

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

