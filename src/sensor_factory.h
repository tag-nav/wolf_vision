/**
 * \file sensor_factory.h
 *
 *  Created on: Apr 25, 2016
 *      \author: jsola
 */

#ifndef SENSOR_FACTORY_H_
#define SENSOR_FACTORY_H_

namespace wolf {
struct IntrinsicsBase;
}

// wolf
#include "wolf.h"

// std
#include <map>

namespace wolf
{

/** \brief Sensor factory class
 *
 * This factory can create objects of classes deriving from SensorBase.
 *
 * Specific object creation is invoked by create(), and the type of sensor is identified with a string.
 * Currently, the following sensor types are implemented,
 *   - "CAMERA" for SensorCamera
 *   - "ODOM 2D" for SensorOdom2D
 *   - "GPS FIX" for SensorGPSFix
 *
 * The rule to make new TYPE strings unique is that you skip the prefix 'Sensor' from your class name,
 * and you build a string in CAPITALS with space separators.
 *
 * #### Accessing the Factory
 * The SensorFactory class is a singleton: it can only exist once in your application.
 * To obtain a pointer to it, use the static method get(),
 *
 *     \code
 *     SensorFactory::get()
 *     \endcode
 *
 * You can then call the methods you like, e.g. to create a sensor, you type:
 *
 *     \code
 *      SensorFactory::get()->create(...); // see below for creating sensors ...
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
 *     SensorFactory::get()->registerCreator("CAMERA", SensorCamera::create);
 *     \endcode
 *
 * The method SensorCamera::create() exists in the SensorCamera class as a static method.
 * All these SensorXxx::create() methods need to have exactly the same API, regardless of the sensor type.
 * This API includes a sensor name, a vector of extrinsic parameters, and a pointer to a base struct of intrinsic parameters, IntrinsicsBase*,
 * that can be derived for each derived sensor.
 *
 * Here is an example of SensorCamera::create() extracted from sensor_camera.h:
 *
 *     \code
 *      static SensorBase* create(std::string& _name, Eigen::VectorXs& _extrinsics_pq, IntrinsicsBase* _intrinsics)
 *      {
 *          // decode extrinsics vector
 *          assert(_extrinsics_pq.size() == 7 && "Bad extrinsics vector length. Should be 7 for 3D.");
 *          StateBlock* pos_ptr = new StateBlock(_extrinsics_pq.head(3));
 *          StateBlock* ori_ptr = new StateQuaternion(_extrinsics_pq.tail(4));
 *
 *          // cast instrinsics to good type and extract intrinsic vector
 *          IntrinsicsCamera* intrinsics = (IntrinsicsCamera*)_intrinsics;
 *          StateBlock*       intr_ptr   = new StateBlock(intrinsics->intrinsic_vector);
 *
 *          SensorBase* sen = new SensorCamera( pos_ptr , ori_ptr , intr_ptr , intrinsics->width , intrinsics->height );
 *          sen->setName(_name); // pass the name to the created SensorCamera.
 *          return sen;
 *      }
 *     \endcode
 *
 * See below for achieving automatic registration of your sensors.
 *
 * #### Achieving automatic registration
 * Currently, registering is performed in each specific SensorXxxx source file, sensor_xxxx.cpp.
 * For example, in sensor_camera.cpp we find the line:
 *
 *     \code
 *      const bool registered_camera = SensorFactory::get()->registerCreator("CAMERA", SensorCamera::create);
 *     \endcode
 *
 * which is a static invocation (i.e., it is placed at global scope outside of the SensorCamera class).
 * Therefore, at application level, all sensors that have a .cpp file compiled are automatically registered.
 *
 * #### Unregistering sensor creators
 * The method unregisterCreator() unregisters the SensorXxx::create() method. It only needs to be passed the string of the sensor type.
 *
 *     \code
 *     SensorFactory::get()->unregisterCreator("CAMERA");
 *     \endcode
 *
 * #### Creating sensors
 * Prior to invoking the creation of a sensor of a particular type,
 * you must register the creator for this type into the factory.
 *
 * To create e.g. a SensorCamera, you type:
 *
 *     \code
 *      SensorFactory::get()->create("CAMERA", "Front-left camera", extrinsics, intrinsics_ptr);
 *     \endcode
 *
 * where ABSOLUTELY ALL input parameters are important. In particular, the sensor name "Front-left camera" will be used to identify this camera
 * and to assign it the appropriate processors. DO NOT USE IT WITH DUMMY PARAMETERS!
 *
 * #### Example
 * We finally provide the necessary steps to create a sensor of class SensorCamera in our application:
 *
 *     \code
 *      #include "sensor_factory.h"
 *      #include "sensor_camera.h" // provides SensorCamera
 *
 *      // Note: SensorCamera::create() is already registered, automatically.
 *
 *      // To create a camera, provide:
 *      //    a type = "CAMERA",
 *      //    a name = "Front-left camera",
 *      //    an extrinsics vector, and
 *      //    a pointer to the intrinsics struct:
 *
 *      Eigen::VectorXs   extrinsics(7);        // give it some values...
 *      IntrinsicsCamera  intrinsics({...});    // also fill in the derived struct
 *
 *      SensorBase* camera_1_ptr =
 *          SensorFactory::get()->create ( "CAMERA" , "Front-left camera" , extrinsics , &intrinsics );
 *
 *      // Second camera... with a different name!
 *      extrinsics = ...;
 *      intrinsics = ...;
 *      SensorBase* camera_2_ptr =
 *          SensorFactory::get()->create( "CAMERA" , "Front-right camera" , extrinsics , &intrinsics );
 *     \endcode
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
        static SensorFactory* get(); // Unique point of access

    public: // see http://stackoverflow.com/questions/1008019/c-singleton-design-pattern
        SensorFactory(const SensorFactory&) = delete;
        void operator=(SensorFactory const&) = delete;
    private:
        SensorFactory() { }
        ~SensorFactory() { }
};

} /* namespace wolf */

#endif /* SENSOR_FACTORY_H_ */
