/**
 * \file sensor_factory.h
 *
 *  Created on: Apr 25, 2016
 *      \author: jsola
 */

#ifndef SENSOR_FACTORY_H_
#define SENSOR_FACTORY_H_

#include "wolf.h"

#include <map>

namespace wolf
{

class SensorFactory
{
    public:
        typedef SensorBase* (*CreateSensorCallback)(std::string & _name);
    private:
        typedef std::map<std::string, CreateSensorCallback> CallbackMap;
    public:
        // Returns 'true' if registration was successful
        bool registerSensor(std::string _sensor_type, CreateSensorCallback createFn);
        // Returns 'true' if the _sensor_type was registered before
        bool unregisterSensor(std::string _sensor_type);
        SensorBase* createSensor(std::string _sensor_type, std::string _name);
    private:
        CallbackMap callbacks_;

        // Singleton ---------------------------------------------------
        // This class is a singleton. The code below guarantees this.
    public:
        static SensorFactory* get(); // Unique point of access
    private:
        SensorFactory();
        // Prevent clients from creating a new Singleton
        SensorFactory(const SensorFactory&);
        // Prevent clients from creating a copy of the Singleton
        static SensorFactory* pInstance_;
};

} /* namespace wolf */

#endif /* SENSOR_FACTORY_H_ */
