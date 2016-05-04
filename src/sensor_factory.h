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
 * Registering sensor creators into the factory is done through registerCreator().
 * You provide a sensor type string (above), and a pointer to a static method
 * that knows how to create your specific sensor, e.g.:
 *
 *     registerCreator("CAMERA", SensorCamera::create);
 *
 * The method SensorCamera::create() exists in the SensorCamera class as a static method.
 * All these create() methods in all SensorXxx classes need to have exactly the same API, regardless of the sensor type.
 * This API includes a sensor name, a vector of extrinsic parameters, and a pointer to a base struct of intrinsic parameters, IntrinsicsBase*,
 * that must be derived for each derived sensor.
 *
 * Here is an example of SensorCamera::create() extracted from sensor_camera.h:
 *
 *      static SensorBase* create(std::string& _name, Eigen::VectorXs& _extrinsics_pq, IntrinsicsBase* _intrinsics)
 *      {
 *          // decode extrinsics vector
 *          assert(_extrinsics_pq.size() == 7 && "Bad extrinsics vector length. Should be 7 for 3D.");
 *          StateBlock* pos_ptr = new StateBlock(_extrinsics_pq.head(3));
 *          StateBlock* ori_ptr = new StateQuaternion(_extrinsics_pq.tail(4));
 *
 *          // cast instrinsics to good type and extract intrinsic vector
 *          IntrinsicsCamera* intrinsics = (IntrinsicsCamera*)_intrinsics;
 *          StateBlock* intr_ptr = new StateBlock(intrinsics->intrinsic_vector);
 *
 *          SensorBase* sen = new SensorCamera(pos_ptr, ori_ptr, intr_ptr, intrinsics->width, intrinsics->height);
 *          sen->setName(_name); // pass the name to the created SensorCamera.
 *          return sen;
 *      }
 *
 * The method unregisterCreator() unregisters the SensorXxx::create() method. It only needs to be passed the string of the sensor type.
 *
 * The SensorFactory class is a singleton: it can only exist once in your application.
 * To obtain a pointer to it, use the static method get(),
 *
 *     SensorFactory::get()
 *
 * You can then call the methods you like, e.g. to create a SensorCamera, you type:
 *
 *      SensorFactory::get()->create("CAMERA", "Front-left camera", extrinsics, intrinsics_ptr);
 *
 * where ABSOLUTELY ALL input parameters are important. In particular, the sensor name "Front-left camera" will be used to identify this camera
 * and to assign it the appropriate processors. DO NOT USE IT WITH DUMMY PARAMETERS!
 *
 * Currently, registering is performed in each specific SensorXxxx header file, sensor_xxxx.h.
 * For example, in sensor_camera.h we find the line:
 *
 *      const bool registered_camera = SensorFactory::get()->registerCreator("CAMERA", SensorCamera::create);
 *
 * which is a static invocation (i.e., it is placed at global scope outside of the SensorCamera class).
 * Therefore, at application level, ````#include````-ing the header file sensor_xxx.h is enough for registering the SensorXxx creator into the factory.
 *
 * We finally provide the necessary steps to create a sensor of class SensorCamera in our application:
 *
 *      #include "sensor_factory.h"
 *      #include "sensor_camera.h" // provides SensorCamera and registers the pair <"CAMERA", SensorCamera::create>
 *
 *      // Note: SensorCamera::create() is already registered. To invoke it, use the std::string "CAMERA" as in the lines below.
 *
 *      // We create two cameras...
 *
 *      // To create a camera, provide a "TYPE", a "Name", an extrinsics vector, and a pointer to the intrinsics struct:
 *      Eigen::VectorXs extrinsics(7); // give it some values...
 *      IntrinsicsCamera intrinsics({...}); // also fill in the derived struct (here we suggested the struct initializer with '{}')...
 *      SensorFactory::get()->create("CAMERA", "Front-left camera", extrinsics, &intrinsics);
 *
 *      // Second camera... with a different name!
 *      extrinsics = ...; // give it some other values...
 *      intrinsics = ...; // idem...
 *      SensorFactory::get()->create("CAMERA", "Front-right camera", extrinsics, &intrinsics);
 *
 * You can also check the code in the example file ````src/examples/test_sensor_factory.cpp````.
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
        SensorBase* create(const std::string& _sensor_type, const std::string& _unique_name, const Eigen::VectorXs& _extrinsics, const IntrinsicsBase* _intrinsics);
    private:
        CallbackMap callbacks_;

        // Singleton ---------------------------------------------------
        // This class is a singleton. The code below guarantees this.
    public:
        static SensorFactory* get(); // Unique point of access
    private:
        SensorFactory(); // Prevent clients from creating a new Singleton
        SensorFactory(const SensorFactory&); // Prevent clients from creating a copy of the Singleton
        ~SensorFactory(); // Prevent clients from deleting the Singleton
};

} /* namespace wolf */

#endif /* SENSOR_FACTORY_H_ */
