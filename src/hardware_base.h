
#ifndef HARDWARE_BASE_H_
#define HARDWARE_BASE_H_

//Wolf includes
#include "wolf.h"
#include "wolf_problem.h"
#include "node_linked.h"
#include "node_terminus.h"
#include "sensor_base.h"

//class HardwareBase
class HardwareBase : public NodeLinked<WolfProblem,SensorBase>
{
    public:
        /** \brief Constructor
         *
         * Constructor
         *
         **/
		HardwareBase();

        /** \brief Destructor
         *
         * Destructor
         *
         **/        
        ~HardwareBase();
        
        /** \brief Adds a sensor
		 *
		 * Adds a sensor
		 *
		 **/
        virtual void addSensor(SensorBase* _sensor_ptr);

        /** \brief Removes a sensor
		 *
		 * Removes a sensor
		 *
		 **/
        void removeSensor(const SensorBaseIter& _sensor_iter);

        /** \brief Removes a sensor
		 *
		 * Removes a sensor
		 *
		 **/
        void removeSensor(SensorBase* _sensor_ptr);

        /** \brief Returns Frame list
         * 
         * Returns FrameBase list
         * 
         **/
        SensorBaseList* getSensorListPtr();
        
};
#endif
