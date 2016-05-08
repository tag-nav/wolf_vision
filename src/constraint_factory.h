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
        ConstraintBase* create(const std::string& _constraint_type, FeatureBase* _feature, NodeBase* _correspondant, ConstraintParamsBase* _params = nullptr);
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
