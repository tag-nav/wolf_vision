#ifndef HARDWARE_BASE_H_
#define HARDWARE_BASE_H_

// Fwd dependencies
namespace wolf{
class SensorBase;
class Problem;
}

//Wolf includes
#include "wolf.h"
#include "node_linked.h"

namespace wolf {

//class HardwareBase
class HardwareBase : public NodeLinked<Problem, SensorBase>
{
    public:
        HardwareBase();

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~HardwareBase();

        virtual SensorBase* addSensor(SensorBase* _sensor_ptr);

        void removeSensor(const SensorBaseIter& _sensor_iter);

        void removeSensor(SensorBase* _sensor_ptr);

        SensorBaseList* getSensorListPtr();

};

} // namespace wolf

// IMPLEMENTATION

#include "sensor_base.h"

namespace wolf {

inline void HardwareBase::removeSensor(const SensorBaseIter& _sensor_iter)
{
    removeDownNode(_sensor_iter);
}

inline SensorBaseList* HardwareBase::getSensorListPtr()
{
    return getDownNodeListPtr();
}

} // namespace wolf

#endif
