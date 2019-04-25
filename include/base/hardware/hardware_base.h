#ifndef HARDWARE_BASE_H_
#define HARDWARE_BASE_H_

// Fwd dependencies
namespace wolf{
class Problem;
class SensorBase;
}

//Wolf includes
#include "base/common/wolf.h"
#include "base/common/node_base.h"

namespace wolf {

//class HardwareBase
class HardwareBase : public NodeBase, public std::enable_shared_from_this<HardwareBase>
{
    private:
        SensorBasePtrList sensor_list_;

    public:
        HardwareBase();
        virtual ~HardwareBase();

        virtual SensorBasePtr addSensor(SensorBasePtr _sensor_ptr);
        SensorBasePtrList& getSensorList();
};

} // namespace wolf

// IMPLEMENTATION

namespace wolf {

inline SensorBasePtrList& HardwareBase::getSensorList()
{
    return sensor_list_;
}

} // namespace wolf

#endif
