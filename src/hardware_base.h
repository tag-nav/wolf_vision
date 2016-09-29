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
        Problem* problem_ptr_;
        SensorBaseList sensor_list_;

    public:
        HardwareBase();

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~HardwareBase();
        void destruct()
        {
            // TODO fill code (see TrajectoryBase::destruct() )
            if (!is_deleting_)
                delete this;
        }

        virtual SensorBase* addSensor(SensorBase* _sensor_ptr);

        void removeSensor(const SensorBaseIter& _sensor_iter);

        void removeSensor(SensorBase* _sensor_ptr);

        SensorBaseList* getSensorListPtr();

        Problem* getProblem(){return problem_ptr_;}
        void setProblem(Problem* _prob_ptr){problem_ptr_ = _prob_ptr;}
};

} // namespace wolf

// IMPLEMENTATION

#include "sensor_base.h"

namespace wolf {

inline void HardwareBase::removeSensor(const SensorBaseIter& _sensor_iter)
{
    sensor_list_.erase(_sensor_iter);
    delete * _sensor_iter;
//    removeDownNode(_sensor_iter);
}

inline SensorBaseList* HardwareBase::getSensorListPtr()
{
    return & sensor_list_;
//    return getDownNodeListPtr();
}

} // namespace wolf

#endif
