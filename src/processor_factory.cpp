/**
 * \file processor_factory.cpp
 *
 *  Created on: May 4, 2016
 *      \author: jsola
 */

#include "processor_factory.h"

#include <iostream>

namespace wolf
{

bool ProcessorFactory::registerCreator(const std::string& _processor_type, CreateProcessorCallback createFn)
{
    bool reg = callbacks_.insert(CallbackMap::value_type(_processor_type, createFn)).second;
    if (reg)
        std::cout << "Registered processor " << _processor_type << std::endl;
    else
        std::cout << "Could not register processor " << _processor_type << std::endl;

    return reg;
}

bool ProcessorFactory::unregisterCreator(const std::string& _processor_type)
{
    return callbacks_.erase(_processor_type) == 1;
}

wolf::ProcessorBase* ProcessorFactory::create(const std::string& _processor_type, const std::string& _name, const ProcessorParamsBase* _params)
{
    CallbackMap::const_iterator i = callbacks_.find(_processor_type);
    if (i == callbacks_.end())
    {
        // not found
        throw std::runtime_error("Unknown Processor type");
    }
    // Invoke the creation function
    return (i->second)(_name, _params);
}

// Singleton ---------------------------------------------------
// This class is a singleton. The code below guarantees this.

wolf::ProcessorFactory* ProcessorFactory::get() // Unique point of access;
{
    static ProcessorFactory pInstance_;
    return &pInstance_;
}

// singleton: constructor and destructor are private
ProcessorFactory::ProcessorFactory(const ProcessorFactory&) { }
ProcessorFactory::ProcessorFactory() { }
ProcessorFactory::~ProcessorFactory() { }
} /* namespace wolf */
