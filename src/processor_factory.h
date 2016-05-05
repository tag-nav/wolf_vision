/**
 * \file processor_factory.h
 *
 *  Created on: May 4, 2016
 *      \author: jsola
 */

#ifndef PROCESSOR_FACTORY_H_
#define PROCESSOR_FACTORY_H_

#include "processor_base.h"

namespace wolf
{

class ProcessorFactory
{
    public:
        typedef ProcessorBase* (*CreateProcessorCallback)(const std::string & _unique_name, const ProcessorParamsBase* _params);
    private:
        typedef std::map<std::string, CreateProcessorCallback> CallbackMap;
    public:
        bool registerCreator(const std::string& _processor_type, CreateProcessorCallback createFn);
        bool unregisterCreator(const std::string& _processor_type);
        ProcessorBase* create(const std::string& _processor_type, const std::string& _unique_name, const ProcessorParamsBase* _params);
    private:
        CallbackMap callbacks_;

        // Singleton ---------------------------------------------------
        // This class is a singleton. The code below guarantees this.
        // See: http://stackoverflow.com/questions/1008019/c-singleton-design-pattern
    public:
        static ProcessorFactory* get(); // Unique point of access
    private:
        ProcessorFactory() { }
        ProcessorFactory(const ProcessorFactory&); // Prevent clients from creating a copy of the Singleton
        ~ProcessorFactory() { } // Prevent clients from deleting the Singleton
        void operator=(ProcessorFactory const&); // Don't implement
};

} /* namespace wolf */

#endif /* PROCESSOR_FACTORY_H_ */
