/**
 * \file sensor_factory.h
 *
 *  Created on: Apr 25, 2016
 *      \author: jsola
 */

#ifndef SENSOR_FACTORY_H_
#define SENSOR_FACTORY_H_

namespace wolf
{
class SensorBase;
struct IntrinsicsBase;
}

// wolf
#include "wolf.h"

// std
#include <string>
#include <map>

namespace wolf
{

/** \brief Sensor factory class
 *
 * This factory can create objects of classes deriving from SensorBase.
 *
 * Specific object creation is invoked by ````create(TYPE, params ... )````, and the TYPE of sensor is identified with a string.
 * Currently, the following sensor types are implemented,
 *   - "CAMERA" for SensorCamera
 *   - "ODOM 2D" for SensorOdom2D
 *   - "GPS FIX" for SensorGPSFix
 *
 * The rule to make new TYPE strings unique is that you skip the prefix 'Sensor' from your class name,
 * and you build a string in CAPITALS with space separators, e.g.:
 *   - SensorCamera -> ````"CAMERA"````
 *   - SensorLaser2D -> ````"LASER 2D"````
 *   - etc.
 *
 * The methods to create specific sensors are called __creators__.
 * Creators must be registered to the factory before they can be invoked for sensor creation.
 *
 * This documentation shows you how to:
 *   - Access the factory
 *   - Register and unregister creators
 *   - Create sensors
 *   - Write a sensor creator for SensorCamera (example).
 *
 * #### Accessing the factory
 * The SensorFactory class is a <a href="http://stackoverflow.com/questions/1008019/c-singleton-design-pattern#1008289">singleton</a>: it can only exist once in your application.
 * To obtain an instance of it, use the static method get(),
 *
 *     \code
 *     SensorFactory::get()
 *     \endcode
 *
 * You can then call the methods you like, e.g. to create a sensor, you type:
 *
 *     \code
 *      SensorFactory::get().create(...); // see below for creating sensors ...
 *     \endcode
 *
 * #### Registering sensor creators
 * Prior to invoking the creation of a sensor of a particular type,
 * you must register the creator for this type into the factory.
 *
 * Registering sensor creators into the factory is done through registerCreator().
 * You provide a sensor type string (above), and a pointer to a static method
 * that knows how to create your specific sensor, e.g.:
 *
 *     \code
 *     SensorFactory::get().registerCreator("CAMERA", SensorCamera::create);
 *     \endcode
 *
 * The method SensorCamera::create() exists in the SensorCamera class as a static method.
 * All these ````SensorXxx::create()```` methods need to have exactly the same API, regardless of the sensor type.
 * This API includes a sensor name, a vector of extrinsic parameters,
 * and a pointer to a base struct of intrinsic parameters, IntrinsicsBase*,
 * that can be derived for each derived sensor:
 *
 *      \code
 *      static SensorBase* create(std::string& _name, Eigen::VectorXs& _extrinsics_pq, IntrinsicsBase* _intrinsics)
 *      \endcode
 *
 * See further down for an implementation example.
 *
 * #### Achieving automatic registration
 * Currently, registering is performed in each specific SensorXxxx source file, sensor_xxxx.cpp.
 * For example, in sensor_camera.cpp we find the line:
 *
 *     \code
 *      const bool registered_camera = SensorFactory::get().registerCreator("CAMERA", SensorCamera::create);
 *     \endcode
 *
 * which is a static invocation (i.e., it is placed at global scope outside of the SensorCamera class).
 * Therefore, at application level, all sensors that have a .cpp file compiled are automatically registered.
 *
 * #### Unregistering sensor creators
 * The method unregisterCreator() unregisters the SensorXxx::create() method. It only needs to be passed the string of the sensor type.
 *
 *     \code
 *     SensorFactory::get().unregisterCreator("CAMERA");
 *     \endcode
 *
 * #### Creating sensors
 * Note: Prior to invoking the creation of a sensor of a particular type,
 * you must register the creator for this type into the factory.
 *
 * To create e.g. a SensorCamera, you type:
 *
 *     \code
 *      SensorFactory::get().create("CAMERA", "Front-left camera", extrinsics, intrinsics_ptr);
 *     \endcode
 *
 * where ABSOLUTELY ALL input parameters are important. In particular, the sensor name "Front-left camera" will be used to identify this camera
 * and to assign it the appropriate processors. DO NOT USE IT WITH DUMMY PARAMETERS!
 *
 * #### See also
 *  - IntrinsicsFactory: to create intrinsic structs deriving from IntrinsicsBase directly from YAML files.
 *  - ProcessorFactory: to create processors that will be bound to sensors.
 *  - Problem::installSensor() : to install sensors in WOLF Problem.
 *
 * #### Example 1: writing a specific sensor creator
 * Here is an example of SensorCamera::create() extracted from sensor_camera.cpp:
 *
 *     \code
 *      static SensorBase* create(std::string& _name, Eigen::VectorXs& _extrinsics_pq, IntrinsicsBase* _intrinsics)
 *      {
 *          // check extrinsics vector
 *          assert(_extrinsics_pq.size() == 7 && "Bad extrinsics vector length. Should be 7 for 3D.");
 *
 *          // cast instrinsics to good type
 *          IntrinsicsCamera* intrinsics_ptr = (IntrinsicsCamera*) _intrinsics;
 *
 *          // Do create the SensorCamera object, and complete its setup
 *          SensorCamera* sen_ptr = new SensorCamera(_extrinsics_pq, intrinsics_ptr);
 *          sen_ptr->setName(_unique_name);
 *
 *          return sen_ptr;
 *      }
 *     \endcode
 *
 * #### Example 2: registering a sensor creator into the factory
 * Registration can be done manually or automatically. It involves the call to static functions.
 * It is advisable to put these calls within unnamed namespaces.
 *
 *   - __Manual registration__: you control registration at application level.
 *   Put the code either at global scope (you must define a dummy variable for this),
 *      \code
 *      namespace {
 *      const bool registered_camera = SensorFactory::get().registerCreator("CAMERA", SensorCamera::create);
 *      }
 *      main () { ... }
 *      \endcode
 *   or inside your main(), where a direct call is possible:
 *      \code
 *      main () {
 *          SensorFactory::get().registerCreator("CAMERA", SensorCamera::create);
 *          ...
 *      }
 *      \endcode
 *
 *   - __Automatic registration__: registration is performed at library level.
 *   Put the code at the last line of the sensor_xxx.cpp file,
 *      \code
 *      namespace {
 *      const bool registered_camera = SensorFactory::get().registerCreator("CAMERA", SensorCamera::create);
 *      }
 *      \endcode
 *   Automatic registration is recommended in wolf, and implemented in the classes shipped with it.
 *   You are free to comment out these lines and place them wherever you consider it more convenient for your design.
 *
 * #### Example 2: creating sensors
 * We finally provide the necessary steps to create a sensor of class SensorCamera in our application:
 *
 *  \code
 *  #include "sensor_factory.h"
 *  #include "sensor_camera.h" // provides SensorCamera
 *
 *  // Note: SensorCamera::create() is already registered, automatically.
 *
 *  using namespace wolf;
 *  int main() {
 *
 *      // To create a camera, provide:
 *      //    a type = "CAMERA",
 *      //    a name = "Front-left camera",
 *      //    an extrinsics vector, and
 *      //    a pointer to the intrinsics struct:
 *
 *      Eigen::VectorXs   extrinsics_1(7);        // give it some values...
 *      IntrinsicsCamera  intrinsics_1({...});    // see IntrinsicsFactory to fill in the derived struct
 *
 *      SensorBase* camera_1_ptr =
 *          SensorFactory::get().create ( "CAMERA" , "Front-left camera" , extrinsics_1 , &intrinsics_1 );
 *
 *      // A second camera... with a different name!
 *
 *      Eigen::VectorXs   extrinsics_2(7);
 *      IntrinsicsCamera  intrinsics_2({...});
 *
 *      SensorBase* camera_2_ptr =
 *          SensorFactory::get().create( "CAMERA" , "Front-right camera" , extrinsics_2 , &intrinsics_2 );
 *
 *      return 0;
 *  }
 *  \endcode
 *
 * You can also check the code in the example file ````src/examples/test_wolf_factories.cpp````.
 */
class SensorFactory
{
    public:
        typedef SensorBase* (*CreateSensorCallback)(const std::string & _unique_name, const Eigen::VectorXs& _extrinsics, const IntrinsicsBase* _intrinsics);
    private:
        typedef std::map<std::string, CreateSensorCallback> CallbackMap;
    public:
        bool registerCreator(const std::string& _sensor_type, CreateSensorCallback createFn);
        bool unregisterCreator(const std::string& _sensor_type);
        SensorBase* create(const std::string& _sensor_type, const std::string& _unique_name, const Eigen::VectorXs& _extrinsics, const IntrinsicsBase* _intrinsics = nullptr);
    private:
        CallbackMap callbacks_;

        // Singleton ---------------------------------------------------
        // This class is a singleton. The code below guarantees this.
        // See: http://stackoverflow.com/questions/1008019/c-singleton-design-pattern
    public:
        static SensorFactory& get(); // Unique point of access

    public: // see http://stackoverflow.com/questions/1008019/c-singleton-design-pattern
        SensorFactory(const SensorFactory&) = delete;
        void operator=(SensorFactory const&) = delete;
    private:
        SensorFactory() { }
        ~SensorFactory() { }
};

} /* namespace wolf */

#endif /* SENSOR_FACTORY_H_ */
