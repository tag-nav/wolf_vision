/**
 * \file processor_params_factory.h
 *
 *  Created on: May 16, 2016
 *      \author: jsola
 */

#ifndef PROCESSOR_PARAMS_FACTORY_H_
#define PROCESSOR_PARAMS_FACTORY_H_

#include "processor_base.h"

namespace wolf
{

class ProcessorParamsFactory
{
    public:
        typedef ProcessorParamsBase* (*CreateProcessorParamsCallback)(const std::string & _filename);
    private:
        typedef std::map<std::string, CreateProcessorParamsCallback> CallbackMap;
    public:
        bool registerCreator(const std::string& _processor_type, CreateProcessorParamsCallback createFn);
        bool unregisterCreator(const std::string& _processor_type);
        ProcessorParamsBase* create(const std::string& _processor_type, const std::string & _filename = "");
    private:
        CallbackMap callbacks_;

        // Singleton ---------------------------------------------------
        // This class is a singleton. The code below guarantees this.
        // See: http://stackoverflow.com/questions/1008019/c-singleton-design-pattern
    public:
        static ProcessorParamsFactory& get(); // Unique point of access

    public: // see http://stackoverflow.com/questions/1008019/c-singleton-design-pattern
        ProcessorParamsFactory(const ProcessorParamsFactory&) = delete;
        void operator=(ProcessorParamsFactory const&) = delete;
    private:
        ProcessorParamsFactory() { }
        ~ProcessorParamsFactory() { }
};

} /* namespace wolf */

#endif /* PROCESSOR_PARAMS_FACTORY_H_ */
