/**
 * \file node_constrained.h
 *
 *  Created on: 14/03/2016
 *      \author: jsola
 */

#ifndef NODE_CONSTRAINED_H_
#define NODE_CONSTRAINED_H_

#include "node_linked.h"


namespace wolf {

/** \brief Base class for nodes receiving Constraints from Features
 * \author jsola
 *
 * Inherit from this class to build Wolf nodes able to receive constraints.
 *
 * This class keeps a lists of constraints received and defines associated functions
 * for accessing and updating it.
 *
 * Typically, Frames, Features and Landmarks are the only nodes requiring this,
 * so FrameBase, FeatureBase and LandmarkBase are NodeConstrained.
 */
template<class UpperType, class LowerType>
class NodeConstrained : public NodeLinked<UpperType, LowerType>
{
    public:
        NodeConstrained(const NodeLocation _loc, const std::string& _class, const std::string&  _type = "BASE", const std::string&  _name = "") :
                NodeLinked<UpperType, LowerType>(_loc, _class, _type, _name),
                constrained_by_list_({})
        {
        }

        virtual ~NodeConstrained()
        {
        }

        virtual void addConstrainedBy(ConstraintBase* _ctr_ptr)
        {
            constrained_by_list_.push_back(_ctr_ptr);
        }
        virtual void removeConstrainedBy(ConstraintBase* _ctr_ptr)
        {
            constrained_by_list_.remove(_ctr_ptr);
        }
        unsigned int getHits() const
        {
            return constrained_by_list_.size();
        }
        ConstraintBaseList* getConstrainedByListPtr()
        {
            return &constrained_by_list_;
        }

    private:
        ConstraintBaseList constrained_by_list_;

};

} // namespace wolf

#endif /* NODE_CONSTRAINED_H_ */
