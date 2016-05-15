/**
 * \file intrinsics_factory.h
 *
 *  Created on: May 11, 2016
 *      \author: jsola
 */

#ifndef INTRINSICS_FACTORY_H_
#define INTRINSICS_FACTORY_H_

#include "sensor_base.h"

#include <string>
#include <map>

namespace wolf
{

class IntrinsicsFactory
{
    public:
        typedef IntrinsicsBase* (*CreateIntrinsicsCallback)(const std::string & _filename);
    private:
        typedef std::map<std::string, CreateIntrinsicsCallback> CallbackMap;
    public:
        bool registerCreator(const std::string& _sensor_type, CreateIntrinsicsCallback createFn);
        bool unregisterCreator(const std::string& _sensor_type);
        IntrinsicsBase* create(const std::string& _sensor_type, const std::string & _filename = "");
    private:
        CallbackMap callbacks_;

        // Singleton ---------------------------------------------------
        // This class is a singleton. The code below guarantees this.
        // See: http://stackoverflow.com/questions/1008019/c-singleton-design-pattern
    public:
        static IntrinsicsFactory& get(); // Unique point of access

    public: // see http://stackoverflow.com/questions/1008019/c-singleton-design-pattern
        IntrinsicsFactory(const IntrinsicsFactory&) = delete;
        void operator=(IntrinsicsFactory const&) = delete;
    private:
        IntrinsicsFactory() { }
        ~IntrinsicsFactory() { }
};

} /* namespace wolf */

#endif /* INTRINSICS_FACTORY_H_ */
