//Wolf
#include "wolf.h"
#include "problem.h"
#include "sensor_imu.h"
#include "capture_imu.h"
#include "constraint_odom_3D.h"
#include "state_block.h"
#include "state_quaternion.h"
#include "processor_imu.h"

//#define DEBUG_RESULTS

int main(int argc, char** argv)
{
    using namespace wolf;
    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;

    std::cout << std::endl << "==================== test_constraint_imu ======================" << std::endl;


    return 0;
}
