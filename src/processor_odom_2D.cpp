#include "processor_odom_2D.h"
namespace wolf
{


}


// Register in the ProcessorFactory
#include "processor_factory.h"
namespace wolf {
namespace
{
const bool registered_prc_odom_2d = ProcessorFactory::get()->registerCreator("ODOM 2D", ProcessorOdom2D::create);
}
} // namespace wolf
