/**
 * \file processor_factory.h
 *
 *  Created on: May 4, 2016
 *      \author: jsola
 */

#ifndef PROCESSOR_FACTORY_H_
#define PROCESSOR_FACTORY_H_

namespace wolf
{
class ProcessorBase;
struct ProcessorParamsBase;
}

#include <string>
#include <map>

namespace wolf
{
/** \brief Processor factory class
 *
 * This factory can create objects of classes deriving from ProcessorBase.
 *
 * Specific object creation is invoked by create(), and the type of processor is identified with a string.
 * For example, the following processor types are implemented,
 *   - "ODOM 3D" for ProcessorOdom3D
 *   - "ODOM 2D" for ProcessorOdom2D
 *   - "GPS"     for ProcessorGPS
 *
 * The rule to make new TYPE strings unique is that you skip the prefix 'Processor' from your class name,
 * and you build a string in CAPITALS with space separators.
 *
 * Registering processor creators into the factory is done through registerCreator().
 * You provide a processor type string (above), and a pointer to a static method
 * that knows how to create your specific processor, e.g.:
 *
 *     registerCreator("ODOM 3D", ProcessorOdom3D::create);
 *
 * The method ProcessorOdom3D::create() exists in the ProcessorOdom3D class as a static method.
 * All these ProcessorXxx::create() methods need to have exactly the same API, regardless of the processor type.
 * This API includes a processor name, and a pointer to a base struct of parameters, ProcessorParamsBase*,
 * that can be derived for each derived processor.
 *
 * Here is an example of ProcessorOdom3D::create() extracted from processor_odom_3D.h:
 *
 *      static ProcessorBase* create(std::string& _name, ProcessorParamsBase* _params)
 *      {
 *          // cast _params to good type
 *          ProcessorParamsOdom3D* params = (ProcessorParamsOdom3D*)_params;
 *
 *          ProcessorBase* prc = new ProcessorOdom3D(params);
 *          prc->setName(_name); // pass the name to the created ProcessorOdom3D.
 *          return prc;
 *      }
 *
 * The method unregisterCreator() unregisters the ProcessorXxx::create() method.
 * It only needs to be passed the string of the processor type.
 *
 * The ProcessorFactory class is a singleton: it can only exist once in your application.
 * To obtain a pointer to it, use the static method get(),
 *
 *     ProcessorFactory::get()
 *
 * You can then call the methods you like, e.g. to create a ProcessorOdom3D, you type:
 *
 *      ProcessorFactory::get()->create("ODOM 3D", "main odometry", params_ptr);
 *
 * Currently, registering is performed in each specific ProcessorXxxx source file, processor_xxxx.cpp.
 * For example, in processor_odom_3D.cpp we find the line:
 *
 *      const bool registered_odom_3D = ProcessorFactory::get()->registerCreator("ODOM 3D", ProcessorOdom3D::create);
 *
 * which is a static invocation (i.e., it is placed at global scope outside of the ProcessorOdom3D class).
 * Therefore, at application level, all processors that have a .cpp file compiled are automatically registered.
 *
 * We finally provide the necessary steps to create a processor of class ProcessorOdom3D in our application:
 *
 *      #include "processor_factory.h"
 *      #include "processor_odom_3D.h" // provides ProcessorOdom3D
 *
 *      // Note: ProcessorOdom3D::create() is already registered, automatically.
 *
 *      // To create a odometry integrator, provide a type="ODOM 3D", a name="main odometry", and a pointer to the parameters struct:
 *
 *      ProcessorParamsOdom3D  params({...});   // fill in the derived struct (note: ProcessorOdom3D actually has no input params)
 *
 *      ProcessorBase* prc_odometry_ptr =
 *          ProcessorFactory::get()->create ( "ODOM 3D" , "main odometry" , &params );
 *
 * You can also check the code in the example file ````src/examples/test_wolf_factories.cpp````.
 */
class ProcessorFactory
{
    public:
        typedef ProcessorBase* (*CreateProcessorCallback)(const std::string & _unique_name, const ProcessorParamsBase* _params);
    private:
        typedef std::map<std::string, CreateProcessorCallback> CallbackMap;
    public:
        bool registerCreator(const std::string& _processor_type, CreateProcessorCallback createFn);
        bool unregisterCreator(const std::string& _processor_type);
        ProcessorBase* create(const std::string& _processor_type, const std::string& _unique_name, const ProcessorParamsBase* _params = nullptr);
    private:
        CallbackMap callbacks_;

        // Singleton ---------------------------------------------------
        // This class is a singleton. The code below guarantees this.
        // See: http://stackoverflow.com/questions/1008019/c-singleton-design-pattern
    public:
        static ProcessorFactory* get(); // Unique point of access

    public: // see http://stackoverflow.com/questions/1008019/c-singleton-design-pattern
        ProcessorFactory(const ProcessorFactory&) = delete;
        void operator=(ProcessorFactory const&) = delete;
    private:
        ProcessorFactory() { }
        ~ProcessorFactory() { }
};

} /* namespace wolf */

#endif /* PROCESSOR_FACTORY_H_ */
