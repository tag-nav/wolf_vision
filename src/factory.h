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

namespace wolf
{

/** \brief singleton factory
 * \param Typebase          base type of the objects created by the factory
 * \param CreatorCallback   type of the pointer to the creator method
 */
template<class TypeBase>
class Factory
{
    public:
         typedef TypeBase* (*CreatorCallback)(const std::string & _filename); // example of creator callback (see typedefs below)
    private:
        typedef std::map<std::string, CreatorCallback> CallbackMap;
    public:
        bool registerCreator(const std::string& _type, CreatorCallback createFn)
        {
            bool reg = callbacks_.insert(typename CallbackMap::value_type(_type, createFn)).second;
            if (reg)
                std::cout << "Factory : registered " << _type << std::endl;
            else
                std::cout << "Factory :  " << _type << " already registered. Skipping. " << std::endl;

            return reg;
        }
        bool unregisterCreator(const std::string& _type)
        {
            return callbacks_.erase(_type) == 1;
        }
        TypeBase* create(const std::string& _type, const std::string& _filename = "")
        {
            typename CallbackMap::const_iterator i = callbacks_.find(_type);
            if (i == callbacks_.end())
            {
                // not found
                throw std::runtime_error("Unknown type. Possibly you tried to use an unregistered creator.");
            }
            // Invoke the creation function
            std::cout << "Creating params for " << _type << "...";
            TypeBase* p = (i->second)(_filename);
            std::cout << " OK." << std::endl;
            return p;
        }
    private:
        CallbackMap callbacks_;

        // Singleton ---------------------------------------------------
        // This class is a singleton. The code below guarantees this.
        // See: http://stackoverflow.com/questions/1008019/c-singleton-design-pattern
    public:
        // Singleton ---------------------------------------------------
        // This class is a singleton. The code below guarantees this.
        static Factory& get()
        {
            static Factory instance_;
            return instance_;
        }

    public: // see http://stackoverflow.com/questions/1008019/c-singleton-design-pattern
        Factory(const Factory&)         = delete;
        void operator=(Factory const&)  = delete;
    private:
        Factory()  { }
        ~Factory() { }
};

} // namespace wolf

#include "sensor_base.h"
#include "processor_base.h"

namespace wolf{

typedef Factory<IntrinsicsBase>      IntrinsicsFactory;
typedef Factory<ProcessorParamsBase> ProcessorParamsFactory;

} /* namespace wolf */

#endif /* FACTORY_H_ */
