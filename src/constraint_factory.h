/**
 * \file constraint_factory.h
 *
 *  Created on: May 7, 2016
 *      \author: jsola
 */

#ifndef CONSTRAINT_FACTORY_H_
#define CONSTRAINT_FACTORY_H_

namespace wolf
{
class NodeBase;  // Feature, Frame and Landmark are NodeBase
class FeatureBase;
class ConstraintBase;
struct ConstraintParamsBase;
}

#include <string>
#include <map>

namespace wolf
{

/** \brief Constraint factory class
 *
 * This factory can create objects of classes deriving from ConstraintBase.
 *
 * Specific object creation is invoked by create(), and the type of constraint is identified with a string.
 * For example, the following constraint types are implemented,
 *   - "ODOM 2D" for ConstraintOdom2D
 *   - "FIX"     for ConstraintFix
 *
 * The rule to make new TYPE strings unique is that you skip the prefix 'Constraint' from your class name,
 * and you build a string in CAPITALS with space separators.
 *
 * Registering constraint creators into the factory is done through registerCreator().
 * You provide a constraint type string (above), and a pointer to a static method
 * that knows how to create your specific constraint, e.g.:
 *
 *     registerCreator("ODOM 2D", ConstraintOdom2D::create);
 *
 * The method ConstraintOdom2D::create() exists in the ConstraintOdom2D class as a static method.
 * All these ConstraintXxx::create() methods need to have exactly the same API, regardless of the constraint type.
 * This API includes a pointer to the Feature owning the constraint, and a pointer to the node constrained,
 * and a pointer to a base struct of parameters, ConstraintParamsBase*, that can be derived for each derived constraint.
 *
 * Here is an example of ConstraintOdom2D::create() extracted from constraint_odom_2D.h:
 *
 *     static wolf::ConstraintBase* create(FeatureBase* _feature_ptr, //
 *                                         NodeBase* _correspondant_ptr, //
 *                                         ConstraintParamsBase* _params = nullptr)
 *     {
 *         return new ConstraintOdom2D(_feature_ptr, (FrameBase*)_correspondant_ptr);
 *     }
 *
 * The method unregisterCreator() unregisters the ConstraintXxx::create() method.
 * It only needs to be passed the string of the constraint type.
 *
 * The ConstraintFactory class is a singleton: it can only exist once in your application.
 * To obtain a pointer to it, use the static method get(),
 *
 *     ConstraintFactory::get()
 *
 * You can then call the methods you like, e.g. to create a ConstraintOdom2D, you type:
 *
 *      ConstraintFactory::get()->create( "ODOM 2D" , feat_ptr , corr_ptr , params_ptr );
 *
 * Currently, registering is performed in each specific ConstraintXxxx source file, constraint_xxxx.cpp.
 * For example, in constraint_odom_2D.cpp we find the line:
 *
 *      const bool registered_odom_2D = ConstraintFactory::get()->registerCreator("ODOM 2D", ConstraintOdom2D::create);
 *
 * which is a static invocation (i.e., it is placed at global scope outside of the ConstraintOdom2D class).
 * Therefore, at application level, all constraints that have a .cpp file compiled are automatically registered.
 *
 * We finally provide the necessary steps to create a constraint of class ConstraintOdom2D in our application:
 *
 *      #include "constraint_factory.h"
 *      #include "constraint_odom_2D.h"     // provides ConstraintOdom2D
 *
 *      // Note: ConstraintOdom2D::create() is already registered, automatically.
 *
 *      // To create a 2D odometry constraint, provide a type="ODOM 2D", a pointer to the parent feature,
 *      // a pointer to the corresponded node (a FrameBase* in this case), and a pointer to the parameters struct:
 *
 *      FeatureOdom2D*          feature_odom_ptr;
 *      FrameBase*              frm_ptr;
 *      ConstraintParamsOdom2D  params({...});   // fill in the derived struct (note: ConstraintOdom2D actually has no input params)
 *
 *      ConstraintBase* prc_odometry_ptr =
 *          ConstraintFactory::get()->create ( "ODOM 2D" , feature_odom_ptr , frm_ptr , &params );
 *
 * You can also check the code in the example file ````src/examples/test_wolf_factories.cpp````.
 */
class ConstraintFactory
{
    public:
        // TODO: Use const in _feature --> need to change aaaallll the ConstraintXxxs' APIs !
        typedef ConstraintBase* (*CreateConstraintCallback)(FeatureBase* _feature, NodeBase* _correspondant, ConstraintParamsBase* _params);
    private:
        typedef std::map<std::string, CreateConstraintCallback> CallbackMap;
    public:
        bool registerCreator(const std::string& _constraint_type, CreateConstraintCallback createFn);
        bool unregisterCreator(const std::string& _constraint_type);
        ConstraintBase* create(const std::string& _constraint_type, FeatureBase* _feature, NodeBase* _correspondant = nullptr, ConstraintParamsBase* _params = nullptr);
    private:
        CallbackMap callbacks_;

        // Singleton ---------------------------------------------------
        // This class is a singleton. The code below guarantees this.
        // See: http://stackoverflow.com/questions/1008019/c-singleton-design-pattern
    public:
        static ConstraintFactory* get(); // Unique point of access

    public: // see http://stackoverflow.com/questions/1008019/c-singleton-design-pattern
        ConstraintFactory(const ConstraintFactory&) = delete;
        void operator=(ConstraintFactory const&) = delete;
    private:
        ConstraintFactory() { }
        ~ConstraintFactory() { }
};

} /* namespace wolf */

#endif /* CONSTRAINT_FACTORY_H_ */
