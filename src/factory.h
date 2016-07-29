/**
 * \file factory.h
 *
 *  Created on: May 16, 2016
 *      \author: jsola
 */

#ifndef FACTORY_H_
#define FACTORY_H_

// std
#include <string>
#include <map>
#include <iostream>
#include <iomanip>

// yaml
#include <yaml-cpp/yaml.h>

namespace wolf
{

/** \brief Singleton template factory
 * \param TypeBase          base type of all the objects created by the factory
 * \param TypeInput         type of the input argument. Typical cases are std::string for file names, and YAML::Node for YAML nodes.
 *
 * This class implements generic factory as a singleton.
 *
 * IMPORTANT: This template factory can be used for many different objects except:
 *   - Objects deriving from SensorBase --> see SensorFactory
 *   - Objects deriving from ProcessorBase --> see ProcessorFactory
 * the reasonf for this is that these two cases above need a more elaborated API than the one in this template class.
 *
 * The class is templatized on the class of the produced objects, TypeBase.
 * The produced objects are always of a class deriving from TypeBase.
 * For example, you may use as TypeBase the following types:
 *   - LandmarkBase: the Factory creates landmarks deriving from LandmarkBase and returns base pointers LandmarkBase* to them
 *   - IntrinsicsBase: the Factory creates intrinsic parameters deriving from IntrinsicsBase and returns base pointers IntrinsicsBase* to them
 *   - XxxBase: the Factory creates objects deriving from XxxBase and returns pointers XxxBase* to them.
 * The returned data is always a pointer to TypeBase.
 *
 * The class in also templatized on the type of the input parameter of the creator, TypeInput:
 *   - std::string is used when the input parameter is to be a file name from which to read data.
 *   - YAML::Node is used when the input parameter is a YAML node with structured data.
 *
 * ### Operation of the factory
 *
 * #### Rationale
 *
 * This factory can create objects of classes deriving from TypeBase.
 *
 * Specific object creation is invoked by ````create(TYPE, params ... )````, and the TYPE of object is identified with a string.
 * For example,
 *   - "CAMERA" for TypeBase = IntrinsicsBase and derived object of type IntrinsicsCamera
 *
 * The rule to make new TYPE strings unique is that you skip the generic 'Type' prefix from your class name,
 * and you build a string in CAPITALS with space separators, e.g.:
 *   - IntrinsicsCamera -> ````"CAMERA"````
 *   - IntrinsicsLaser2D -> ````"LASER 2D"````
 *   - LandmarkPolyline2D -> ````"POLYLINE 2D"````
 *   - etc.
 *
 * The methods to create specific objects are called __creators__.
 * Creators must be registered to the factory before they can be invoked for object creation.
 *
 * This documentation shows you how to:
 *   - Access the factory
 *   - Register and unregister creators
 *   - Create objects
 *   - Write a object creator for LandmarkPolyline2D (example).
 *
 * #### Accessing the factory
 * The Factory class is a <a href="http://stackoverflow.com/questions/1008019/c-singleton-design-pattern#1008289">singleton</a>: it can only exist once in your application.
 * To obtain an instance of it, use the static method get(),
 *
 *     \code
 *     Factory<MyTypeBase, MyTypeInput>::get()
 *     \endcode
 *
 * You can then call the methods you like, e.g. to create an object, you type:
 *
 *     \code
 *      Factory<MyTypeBase, MyTypeInput>::get().create(...); // see below for creating objects ...
 *     \endcode
 *
 * #### Registering object creators
 * Prior to invoking the creation of an object of a particular type,
 * you must register the creator for this type into the factory.
 *
 * Registering object creators into the factory is done through registerCreator().
 * You provide an object type string (above), and a pointer to a static method
 * that knows how to create your specific object, e.g.:
 *
 *     \code
 *     Factory<MyTypeBase, MyTypeInput>::get().registerCreator("POLYLINE 2D", LandmarkPolyline2D::create);
 *     \endcode
 *
 * The method LandmarkPolyline2D::create(...) exists in the SensorCamera class as a static method.
 * All these ````XxxXxx::create()```` methods need to have exactly the same API, regardless of the object type.
 * This API includes an element of type TypeInput, which might be either a std::string, or a YAML::node:
 *   - filenames are used to access YAML files with configuration data to create your object
 *   - YAML::Node are used to access parts of a YAML file already encoded as nodes, such as when loading landmarks from a SLAM map ````MapBase````.
 * Two examples:
 *
 *      \code
 *      static IntrinsicsBase* create(const std::string& _intrinsics_dot_yaml)
 *      static LandmarkBase*   create(const YAML::Node& _lmk_yaml_node)
 *      \endcode
 *
 * See further down for an implementation example.
 *
 * #### Achieving automatic registration
 * Currently, registering is performed in specific source files, object_xxxx.cpp.
 * For example, in sensor_camera_yaml.cpp we find the line:
 *
 *     \code
 *     const bool registered_camera_intr = IntrinsicsFactory::get().registerCreator("CAMERA", createIntrinsicsCamera);
 *     \endcode
 *
 * which is a static invocation (i.e., it is placed at global scope outside of the SensorCamera class).
 * Therefore, at application level, all objects that have a .cpp file compiled are automatically registered.
 *
 * #### Unregistering object creators
 * The method unregisterCreator() unregisters the ObjectXxx::create() method. It only needs to be passed the string of the object type.
 *
 *     \code
 *     Factory<MyTypeBase, MyTypeInput>::get().unregisterCreator("CAMERA");
 *     \endcode
 *
 * #### Creating objects
 * Note: Prior to invoking the creation of a object of a particular type,
 * you must register the creator for this type into the factory.
 *
 * To create e.g. a LandmarkPolyline2D from a YAML node you type:
 *
 *     \code
 *      Factory<LandmarkBase*, YAML::Node>::get().create("POLYLINE 2D", lmk_yaml_node);
 *     \endcode
 *
 * or even better, make use of the convenient typedefs, ````typedef Factory<LandmarkBase*, YAML::Node> LandmarkFactory````:
 *
 *     \code
 *      LandmarkFactory::get().create("POLYLINE 2D", lmk_yaml_node);
 *     \endcode
 *
 * #### See also
 *  - IntrinsicsFactory: typedef of this template to create intrinsic structs deriving from IntrinsicsBase directly from YAML files.
 *  - ProcessorParamsFactory: typedef of this template to create processor params structs deriving from ProcessorParamsBase directly from YAML files.
 *  - LandmarkFactory: typedef of this template to create landmarks deriving from LandmarkBase directly from YAML nodes.
 *  - SensorFactory: to create sensors
 *  - Problem::loadMap() : to load a maps directly from YAML files.
 *  - ProcessorFactory: to create processors that will be bound to objects.
 *  - Problem::installSensor() : to install objects in WOLF Problem.
 *
 * #### Example 1: Writing and registering the creator of LandmarkPolyline2D from a YAML node
 *
 * You can find this code in the landmark_polyline_2D.cpp file.
 *
 * \code
 *  // Creator (this method is static):
 * LandmarkBase* LandmarkPolyline2D::create(const YAML::Node& _lmk_node)
 * {
 *    // Parse YAML node with lmk info and data
 *    unsigned int id         = _lmk_node["id"].as<unsigned int>();
 *    int first_id            = _lmk_node["first_id"].as<int>();
 *    bool first_defined      = _lmk_node["first_defined"].as<bool>();
 *    bool last_defined       = _lmk_node["last_defined"].as<bool>();
 *    unsigned int npoints    = _lmk_node["points"].size();
 *    Eigen::MatrixXs points(2,npoints);
 *    for (unsigned int i = 0; i < npoints; i++)
 *    {
 *        points.col(i) = _lmk_node["points"][i].as<Eigen::Vector2s>();
 *    }
 *
 *    // Create a new landmark
 *    LandmarkBase* lmk_ptr = new LandmarkPolyline2D(points, first_defined, last_defined, first_id);
 *    lmk_ptr->setId(id);
 *
 *    return lmk_ptr;
 * }
 *
 * // Register landmark creator (put the register code inside an unnamed namespace):
 *  namespace
 *  {
 *  const bool registered_lmk_polyline_2D = LandmarkFactory::get().registerCreator("POLYLINE 2D", LandmarkPolyline2D::create);
 *  }
 *
 * \endcode
 *
 *
 * You can also check the code in the example file ````src/examples/test_map_yaml.cpp````.
 */
template<class TypeBase, class TypeInput>
class Factory
{
    public:
        typedef TypeBase* (*CreatorCallback)(const TypeInput & _input); // example of creator callback (see typedefs below)
    private:
        typedef std::map<std::string, CreatorCallback> CallbackMap;
    public:
        bool registerCreator(const std::string& _type, CreatorCallback createFn);
        bool unregisterCreator(const std::string& _type);
        TypeBase* create(const std::string& _type, const TypeInput& _input = "");
        std::string getClass();

    private:
        CallbackMap callbacks_;

        // Singleton ---------------------------------------------------
        // This class is a singleton. The code below guarantees this.
        // See: http://stackoverflow.com/questions/1008019/c-singleton-design-pattern
    public:
        static Factory& get();
    public:
        Factory(const Factory&)         = delete;
        void operator=(Factory const&)  = delete;
    private:
        Factory()  { }
        ~Factory() { }
};

template<class TypeBase, class TypeInput>
inline bool Factory<TypeBase, TypeInput>::registerCreator(const std::string& _type, CreatorCallback createFn)
{
    bool reg = callbacks_.insert(typename CallbackMap::value_type(_type, createFn)).second;
    if (reg)
        std::cout << std::setw(22) << std::left << getClass() << " : registered " << _type << std::endl;
    else
        std::cout << getClass() << " :  " << _type << " already registered. Skipping. " << std::endl;

    return reg;
}

template<class TypeBase, class TypeInput>
inline bool Factory<TypeBase, TypeInput>::unregisterCreator(const std::string& _type)
{
    return callbacks_.erase(_type) == 1;
}

template<class TypeBase, class TypeInput>
inline TypeBase* Factory<TypeBase, TypeInput>::create(const std::string& _type, const TypeInput& _input)
{
    typename CallbackMap::const_iterator creator_callback_it = callbacks_.find(_type);
    if (creator_callback_it == callbacks_.end())
        // not found
        throw std::runtime_error("Unknown type. Possibly you tried to use an unregistered creator.");
    // Invoke the creation function
    TypeBase* p = (creator_callback_it->second)(_input);
    return p;
}

template<class TypeBase, class TypeInput>
inline Factory<TypeBase, TypeInput>& Factory<TypeBase, TypeInput>::get()
{
    static Factory instance_;
    return instance_;
}

template<class TypeBase, class TypeInput>
inline std::string Factory<TypeBase, TypeInput>::getClass()
{
    return "Factory<class TypeBase>";
}

} // namespace wolf


// Some specializations

#include "sensor_base.h"
#include "processor_base.h"
#include "landmark_base.h"

namespace wolf
{

typedef Factory<IntrinsicsBase, std::string>        IntrinsicsFactory;
typedef Factory<ProcessorParamsBase, std::string>   ProcessorParamsFactory;
typedef Factory<LandmarkBase, YAML::Node>           LandmarkFactory;

template<>
inline std::string Factory<IntrinsicsBase, std::string>::getClass()
{
    return "IntrinsicsFactory";
}

template<>
inline std::string Factory<ProcessorParamsBase, std::string>::getClass()
{
    return "ProcessorParamsFactory";
}

template<>
inline std::string Factory<LandmarkBase, YAML::Node>::getClass()
{
    return "LandmarkFactory";
}


} /* namespace wolf */

#endif /* FACTORY_H_ */
