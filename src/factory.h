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
 *
 * This class implements a generic factory as a singleton.
 *
 * > IMPORTANT: This template factory can be used to construct many different objects except:
 * >   - Objects deriving from SensorBase --> see SensorFactory
 * >   - Objects deriving from ProcessorBase --> see ProcessorFactory
 * >
 * > The reason for this is that the two cases above need a more elaborated API than the one in this template class.
 *
 * \param TypeBase          base type of all the objects created by the factory
 * \param TypeInput         type of the input argument. Typical cases are std::string for file names, and YAML::Node for YAML nodes.
 *
 * - The class is templatized on the class of the produced objects, __TypeBase__.
 *   The produced objects are always of a class deriving from TypeBase.
 *   The returned data is always a pointer to TypeBase.
 *
 *   For example, you may use as __TypeBase__ the following types:
 *     - LandmarkBase: the Factory creates landmarks deriving from LandmarkBase and returns base pointers ````LandmarkBasePtr```` to them
 *     - IntrinsicsBase: the Factory creates intrinsic parameters deriving from IntrinsicsBase and returns base pointers ````IntrinsicsBasePtr```` to them
 *     - XxxBase: the Factory creates objects deriving from XxxBase and returns pointers ````XxxBasePtr```` to them.
 *
 * - The class in also templatized on the type of the input parameter of the creator, __TypeInput__:
 *   - ````std::string```` is used when the input parameter is a file name from which to read data (typically a YAML file).
 *   - ````YAML::Node```` is used when the input parameter is a YAML node with structured data.
 *
 * ### Operation of the factory
 *
 * #### Rationale
 *
 * This factory can create objects of classes deriving from TypeBase.
 *
 * > For example, if you want to make a Landmark factory, set TypeBase = LandmarkBase.\n
 * > Then, the factory will create specific landmarks deriving from LandmarkBase.\n
 * > The specific type of landmark (e.g. LandmarkCorner2D, LandmarkAHP, LandmarkPolyline2D, etc) is indicated by a string that we call TYPE in this documentation.
 *
 * Specific object creation is invoked by the method ````create(TYPE, params ... )````, where
 *   - the TYPE of object to create is identified with a string
 *   - the params may be provided in different forms -- see TypeInput.
 *
 * The methods to create specific objects are called __creators__.
 * Creators must be registered to the factory before they can be invoked for object creation.
 *
 * This documentation shows you how to:
 *   - Define correct TYPE names
 *   - Access the factory
 *   - Write object creators
 *   - Register and unregister object creators to the factory
 *   - Create objects using the factory
 *   - Examples: Write and register a landmark creator for LandmarkPolyline2D.
 *
 * #### Define correct TYPE names
 * The rule to make new TYPE strings unique is that you skip the generic 'Type' prefix from your class name,
 * and you build a string in CAPITALS with space separators, e.g.:
 *   - IntrinsicsCamera -> ````"CAMERA"````
 *   - IntrinsicsLaser2D -> ````"LASER 2D"````
 *   - LandmarkPolyline2D -> ````"POLYLINE 2D"````
 *   - etc.
 *
 * #### Access the factory
 * The first thing to know is that we have defined typedefs for the templates that we are using. For example:
 *
 * \code
 * typedef Factory<IntrinsicsBase, std::string>        IntrinsicsFactory;
 * typedef Factory<ProcessorParamsBase, std::string>   ProcessorParamsFactory;
 * typedef Factory<LandmarkBase, YAML::Node>           LandmarkFactory;
 * \endcode
 *
 * Second to know, the Factory class is a <a href="http://stackoverflow.com/questions/1008019/c-singleton-design-pattern#1008289">singleton</a>: it can only exist once in your application.
 * To access it, use the static method get(),
 *
 *     \code
 *     Factory<MyTypeBase, MyTypeInput>::get()
 *     \endcode
 *
 * where, of course, you better make use of the appropriate typedef in place of ````Factory<MyTypeBase, MyTypeInput>````.
 *
 * You can then call the methods you like, e.g. to create a landmark, you use:
 *
 *     \code
 *     LandmarkFactory::get().create(...); // see below for creating objects ...
 *     \endcode
 *
 * #### Write creator methods (in your derived object classes)
 * The method LandmarkPolyline2D::create(...) exists in the LandmarkPolyline2D class as a static method.
 * All these ````XxxXxx::create()```` methods need to have exactly the same API, regardless of the object type.
 * The API puts into play the two template parameters:
 *
 * \code
 * static TypeBase* create( const TypeInput& );
 * \endcode
 *
 * This API includes an element of type TypeInput, which might be either a std::string, or a YAML::node:
 *   - ````std::string```` is used to indicate the name of a configuration file. These files are usually YAML files containing configuration data to create your object.
 *   - ````YAML::Node```` is used to access parts of a YAML file already encoded as nodes, such as when loading landmarks from a SLAM map stored as a YAML file.
 *
 * Two examples:
 *
 *      \code
 *      static IntrinsicsBasePtr create(const std::string& _intrinsics_dot_yaml)
 *      static LandmarkBasePtr   create(const YAML::Node& _lmk_yaml_node)
 *      \endcode
 *
 * See further down for an implementation example.
 *
 * #### Register object creators
 * Prior to invoking the creation of an object of a particular TYPE,
 * you must register the creator for this type into the factory.
 *
 * Registering object creators into the factory is done through registerCreator().
 * You provide an object TYPE string (above), and a pointer to a static method
 * that knows how to create your specific object, e.g.:
 *
 *     \code
 *     LandmarkFactory::get().registerCreator("POLYLINE 2D", LandmarkPolyline2D::create);
 *     \endcode
 *
 * #### Automatic registration
 * Currently, registering is performed in specific source files, object_xxxx.cpp.
 * For example, in sensor_camera_yaml.cpp we find the line:
 *
 *     \code
 *     const bool registered_camera_intr = IntrinsicsFactory::get().registerCreator("CAMERA", createIntrinsicsCamera);
 *     \endcode
 *
 * which is a static invocation (i.e., it is placed at global scope outside of the IntrinsicsCamera class).
 *
 * Therefore, at application level, all objects that have a .cpp file compiled are automatically registered.
 *
 * #### Unregister object creators
 * The method unregisterCreator() unregisters the ObjectXxx::create() method. It only needs to be passed the string of the object type.
 *
 *     \code
 *     Factory<MyTypeBase, MyTypeInput>::get().unregisterCreator("CAMERA");
 *     \endcode
 *
 * #### Create objects using the factory
 * Note: Prior to invoking the creation of a object of a particular type,
 * you must register the creator for this type into the factory.
 *
 * To create e.g. a LandmarkPolyline2D from a YAML node you type:
 *
 *     \code
 *     LandmarkBasePtr lmk_ptr = Factory<LandmarkBasePtr, YAML::Node>::get().create("POLYLINE 2D", lmk_yaml_node);
 *     \endcode
 *
 * or even better, make use of the convenient typedefs:
 *
 *     \code
 *     LandmarkBasePtr lmk_ptr = LandmarkFactory::get().create("POLYLINE 2D", lmk_yaml_node);
 *     \endcode
 *
 * ### Examples
 * #### Example 1: Writing the creator of LandmarkPolyline2D from a YAML node
 *
 * You can find this code in the landmark_polyline_2D.cpp file.
 *
 * \code
 *  // Creator (this method is static):
 * LandmarkBasePtr LandmarkPolyline2D::create(const YAML::Node& _lmk_node)
 * {
 *    // Parse YAML node with lmk info and data
 *    unsigned int      id              = _lmk_node["id"].as<unsigned int>();
 *    int               first_id        = _lmk_node["first_id"].as<int>();
 *    bool              first_defined   = _lmk_node["first_defined"].as<bool>();
 *    bool              last_defined    = _lmk_node["last_defined"].as<bool>();
 *    unsigned int      npoints         = _lmk_node["points"].size();
 *    Eigen::MatrixXs   points(2,npoints);
 *    for (unsigned int i = 0; i < npoints; i++)
 *    {
 *        points.col(i)                 = _lmk_node["points"][i].as<Eigen::Vector2s>();
 *    }
 *
 *    // Create a new landmark
 *    LandmarkBasePtr lmk_ptr = new LandmarkPolyline2D(points, first_defined, last_defined, first_id);
 *    lmk_ptr->setId(id);
 *
 *    return lmk_ptr;
 * }
 * \endcode
 *
 * #### Example 2: Registering the creator of LandmarkPolyline2D from a YAML node
 *
 * You can find this code in the landmark_polyline_2D.cpp file.
 *
 * \code
 * // Register landmark creator (put the register code inside an unnamed namespace):
 *  namespace
 *  {
 *  const bool registered_lmk_polyline_2D = LandmarkFactory::get().registerCreator("POLYLINE 2D", LandmarkPolyline2D::create);
 *  }
 *
 * \endcode
 *
 * ### More information
 *  - IntrinsicsFactory: typedef of this template to create intrinsic structs deriving from IntrinsicsBase directly from YAML files.
 *  - ProcessorParamsFactory: typedef of this template to create processor params structs deriving from ProcessorParamsBase directly from YAML files.
 *  - LandmarkFactory: typedef of this template to create landmarks deriving from LandmarkBase directly from YAML nodes.
 *  - Problem::loadMap() : to load a maps directly from YAML files.
 *  - You can also check the code in the example file ````src/examples/test_map_yaml.cpp````.
 *
 * #### See also
 *  - SensorFactory: to create sensors
 *  - ProcessorFactory: to create processors.
 *  - Problem::installSensor() : to install sensors in WOLF Problem.
 *  - Problem::installProcessor() : to install processors in WOLF Problem.
 *
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
