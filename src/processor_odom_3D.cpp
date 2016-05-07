#include "processor_odom_3D.h"
namespace wolf
{


}


// Register in the SensorFactory
#include "processor_factory.h"
namespace wolf {
namespace
{
const bool registered_prc_odom_3d = ProcessorFactory::get()->registerCreator("ODOM 3D", ProcessorOdom3D::create);
}
} // namespace wolf
