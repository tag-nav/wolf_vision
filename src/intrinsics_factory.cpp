/**
 * \file intrinsics_factory.cpp
 *
 *  Created on: May 11, 2016
 *      \author: jsola
 */

#include "intrinsics_factory.h"

#include <iostream>

namespace wolf
{

bool IntrinsicsFactory::registerCreator(const std::string& _intrinsics_type, CreateIntrinsicsCallback createFn)
{
    bool reg = callbacks_.insert(CallbackMap::value_type(_intrinsics_type, createFn)).second;
    if (reg)
        std::cout << "IntrinsicsFactory : registered " << _intrinsics_type << std::endl;
    else
        std::cout << "IntrinsicsFactory : intrinsics " << _intrinsics_type << " already registered. Skipping. " << std::endl;
    return reg;
}

bool IntrinsicsFactory::unregisterCreator(const std::string& _intrinsics_type)
{
    return callbacks_.erase(_intrinsics_type) == 1;
}

IntrinsicsBase* IntrinsicsFactory::create(const std::string& _intrinsics_type, const std::string & _filename_dot_yaml)
{
    CallbackMap::const_iterator i = callbacks_.find(_intrinsics_type);
    if (i == callbacks_.end())
    {
        // not found
        throw std::runtime_error("Unknown Intrinsics type");
    }
    // Invoke the creation function
    return (i->second)(_filename_dot_yaml);
}

// Singleton ---------------------------------------------------
// This class is a singleton. The code below guarantees this.

IntrinsicsFactory* IntrinsicsFactory::get() // Unique point of access;
{
    static IntrinsicsFactory instance_;
    return &instance_;
}

} /* namespace wolf */
