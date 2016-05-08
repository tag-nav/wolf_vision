#include "constraint_fix.h"

//===================================================================================================================
// Register in the SensorFactory
#include "constraint_factory.h"
namespace wolf {
namespace
{
const bool registered_ctr_fix = ConstraintFactory::get()->registerCreator("FIX", ConstraintFix::create);
}
} // namespace wolf
