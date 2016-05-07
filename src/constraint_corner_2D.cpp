#include "constraint_corner_2D.h"


//===================================================================================================================
// Register in the SensorFactory
#include "constraint_factory.h"
namespace wolf {
namespace
{
const bool registered_ctr_corner = ConstraintFactory::get()->registerCreator("CORNER 2D", ConstraintCorner2D::create);
}
} // namespace wolf


