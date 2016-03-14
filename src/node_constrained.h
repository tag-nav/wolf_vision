/**
 * \file node_constrained.h
 *
 *  Created on: 14/03/2016
 *      \author: jsola
 */

#ifndef NODE_CONSTRAINED_H_
#define NODE_CONSTRAINED_H_

#include "node_linked.h"

/** \brief Base class for nodes receiving Constraints from Features
 * \author jsola
 *
 * Inherit from this class to build Wolf nodes able to receive constraints.
 *
 * Typically, Frames, Features and Landmarks are the only nodes requiring this,
 * so FrameBase, FeatureBase and LandmarkBase are NodeConstrained.
 */
template<class UpperType, class LowerType>
class NodeConstrained : public NodeLinked<UpperType, LowerType>
{
    public:
        NodeConstrained(const NodeLocation _loc, const std::string& _label) :
                NodeLinked<UpperType, LowerType>(_loc, _label),
                constrained_by_list_({})
        {
        }

        virtual ~NodeConstrained()
        {
        }

        void addConstrainedBy(ConstraintBase* _ctr_ptr)
        {
            constrained_by_list_.push_back(_ctr_ptr);
        }
        void removeConstrainedBy(ConstraintBase* _ctr_ptr)
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

#endif /* NODE_CONSTRAINED_H_ */
