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

/** \brief singleton template factory
 * \param TypeBase          base type of all the objects created by the factory
 * \param TypeInput         type of the input argument. Typical cases are std::string for file names, and YAML::Node for YAML nodes.
 *
 * This class implements generic factory as a singleton.
 *
 * The class in templatized on the type of the input parameter of the creator:
 *   - std::string is used when the input parameter is to be a file name from which to read data.
 *   - YAML::Node is used when the input parameter is a YAML node with structured data.
 *
 * The class is also templatized on the output class, that is, the type of objects created.
 * These are basically pointers to TypeBase, for example:
 *   - LandmarkBase: the Factory creates landmarks deriving from LandmarkBase and returns base pointers LandmarkBase* to them
 *   - IntrinsicsBase: the Factory creates intrinsic parameters deriving from IntrinsicsBase and returns base pointers IntrinsicsBase* to them
 *   - XxxBase: the Factory creates objects deriving from XxxBase and returns pointers XxxBase* to them
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
