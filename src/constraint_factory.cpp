/**
 * \file constraint_factory.cpp
 *
 *  Created on: May 7, 2016
 *      \author: jsola
 */

#include "constraint_factory.h"

#include <string>
#include <iostream>

namespace wolf {

bool ConstraintFactory::registerCreator(const std::string& _constraint_type, CreateConstraintCallback createFn)
{
    bool reg = callbacks_.insert(CallbackMap::value_type(_constraint_type, createFn)).second;
    if (reg)
        std::cout << "Registered constraint " << _constraint_type << std::endl;
    else
        std::cout << "Constraint " << _constraint_type << " already registered. Skipping. " << std::endl;

    return reg;
}

bool ConstraintFactory::unregisterCreator(const std::string& _constraint_type)
{
    return callbacks_.erase(_constraint_type) == 1;
}

ConstraintBase* ConstraintFactory::create(const std::string& _constraint_type, const NodeBase* _correspondant)
{
    CallbackMap::const_iterator i = callbacks_.find(_constraint_type);
    if (i == callbacks_.end())
    {
        // not found
        throw std::runtime_error("Unknown Constraint type");
    }
    // Invoke the creation function
    return (i->second)(_correspondant);
}

ConstraintFactory* ConstraintFactory::get() // Unique point of access;
{
    static ConstraintFactory instance_; // Guaranteed to be destroyed.
                                        // Instantiated on first use.
    return &instance_;
}


} // namespace wolf
