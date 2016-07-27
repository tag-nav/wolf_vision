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

namespace wolf
{

/** \brief singleton factory
 * \param TypeBase          base type of all the objects created by the factory
 */
template<class TypeBase>
class Factory
{
    public:
         typedef TypeBase* (*CreatorCallback)(const std::string & _filename); // example of creator callback (see typedefs below)
    private:
        typedef std::map<std::string, CreatorCallback> CallbackMap;
    public:
        bool registerCreator(const std::string& _type, CreatorCallback createFn);
        bool unregisterCreator(const std::string& _type);
        TypeBase* create(const std::string& _type, const std::string& _filename = "");
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

template<class TypeBase>
inline bool Factory<TypeBase>::registerCreator(const std::string& _type, CreatorCallback createFn)
{
    bool reg = callbacks_.insert(typename CallbackMap::value_type(_type, createFn)).second;
    if (reg)
        std::cout << std::setw(22) << std::left << getClass() << " : registered " << _type << std::endl;
    else
        std::cout << getClass() << " :  " << _type << " already registered. Skipping. " << std::endl;

    return reg;
}

template<class TypeBase>
inline bool Factory<TypeBase>::unregisterCreator(const std::string& _type)
{
    return callbacks_.erase(_type) == 1;
}

template<class TypeBase>
inline TypeBase* Factory<TypeBase>::create(const std::string& _type, const std::string& _filename)
{
    typename CallbackMap::const_iterator creator_callback_it = callbacks_.find(_type);
    if (creator_callback_it == callbacks_.end())
        // not found
        throw std::runtime_error("Unknown type. Possibly you tried to use an unregistered creator.");
    // Invoke the creation function
    TypeBase* p = (creator_callback_it->second)(_filename);
    return p;
}

template<class TypeBase>
inline Factory<TypeBase>& Factory<TypeBase>::get()
{
    static Factory instance_;
    return instance_;
}

template<class TypeBase>
inline std::string Factory<TypeBase>::getClass()
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

typedef Factory<IntrinsicsBase>      IntrinsicsFactory;
typedef Factory<ProcessorParamsBase> ProcessorParamsFactory;
typedef Factory<LandmarkBase>        LandmarkFactory;

template<>
inline std::string Factory<IntrinsicsBase>::getClass()
{
    return "IntrinsicsFactory";
}

template<>
inline std::string Factory<ProcessorParamsBase>::getClass()
{
    return "ProcessorParamsFactory";
}

template<>
inline std::string Factory<LandmarkBase>::getClass()
{
    return "LandmarkFactory";
}


} /* namespace wolf */

#endif /* FACTORY_H_ */
