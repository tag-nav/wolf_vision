#ifndef HARDWARE_BASE_H_
#define HARDWARE_BASE_H_

// Fwd dependencies
namespace wolf{
class Problem;
class SensorBase;
}

//Wolf includes
#include "wolf.h"
#include "node_base.h"

namespace wolf {

//class HardwareBase
class HardwareBase : public NodeBase //: public NodeLinked<Problem, SensorBase>
{
    private:
        ProblemWPtr problem_ptr_;
        SensorBaseList sensor_list_;

    public:
        HardwareBase();
        virtual ~HardwareBase();

        ProblemPtr getProblem(){return problem_ptr_;}
        void setProblem(ProblemPtr _prob_ptr){problem_ptr_ = _prob_ptr;}

        virtual SensorBasePtr addSensor(SensorBasePtr _sensor_ptr);
        SensorBaseList* getSensorListPtr();
        void removeSensor(const SensorBaseIter& _sensor_iter);
        void removeSensor(SensorBasePtr _sensor_ptr);
};

} // namespace wolf

// IMPLEMENTATION

#include "sensor_base.h"

namespace wolf {

inline void HardwareBase::removeSensor(const SensorBaseIter& _sensor_iter)
{
    sensor_list_.erase(_sensor_iter);
    //    delete * _sensor_iter; // TODO remove line
}

inline SensorBaseList* HardwareBase::getSensorListPtr()
{
    return & sensor_list_;
}

} // namespace wolf

#endif
