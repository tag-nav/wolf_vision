#include "constraint_container.h"

//===================================================================================================================
// Register in the SensorFactory
#include "constraint_factory.h"
namespace wolf {
namespace
{
const bool registered_ctr_container = ConstraintFactory::get()->registerCreator("CONTAINER", ConstraintContainer::create);
}
} // namespace wolf


