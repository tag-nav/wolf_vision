/**
 * \file parameter_factory.h
 *
 *  Created on: May 11, 2016
 *      \author: jsola
 */

#ifndef PARAMETER_FACTORY_H_
#define PARAMETER_FACTORY_H_

namespace wolf
{
struct ParameterBase;
}

#include <string>
#include <map>

namespace wolf
{

class ParameterFactory
{
    public:
        typedef ParameterBase* (*CreateParameterCallback)(const std::string & _filename_dot_yaml);
    private:
        typedef std::map<std::string, CreateParameterCallback> CallbackMap;
    public:
        bool registerCreator(const std::string& _parameter_type, CreateParameterCallback createFn);
        bool unregisterCreator(const std::string& _parameter_type);
        ParameterBase* create(const std::string& _parameter_type, const std::string & _filename_dot_yaml = "");
    private:
        CallbackMap callbacks_;

        // Singleton ---------------------------------------------------
        // This class is a singleton. The code below guarantees this.
        // See: http://stackoverflow.com/questions/1008019/c-singleton-design-pattern
    public:
        static ParameterFactory* get(); // Unique point of access

    public: // see http://stackoverflow.com/questions/1008019/c-singleton-design-pattern
        ParameterFactory(const ParameterFactory&) = delete;
        void operator=(ParameterFactory const&) = delete;
    private:
        ParameterFactory() { }
        ~ParameterFactory() { }
};

} /* namespace wolf */

#endif /* PARAMETER_FACTORY_H_ */
