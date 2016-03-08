#ifndef NODE_BASE_H_
#define NODE_BASE_H_

// Fwd references
class WolfProblem;

// Wolf includes
#include "wolf.h"

// std includes

/** \brief Base class for Nodes
 *
 * Base class for all Nodes in the Wolf tree.
 * It implements the ID factory.
 *
 **/
class NodeBase
{
    protected:
        std::string label_; ///< Text label identifying the node
        unsigned int node_id_; ///< Node id. It is unique over the whole Wolf Tree
        static unsigned int node_id_count_; ///< Object counter (acts as simple ID factory)
        //PendingStatus node_pending_; ///< Pending to be added/updated in the filter or optimizer
        bool verbose_; 

    public: 

        /** \brief Constructor from label.
         *
         * Constructor from label
		 * 
         */
        NodeBase(std::string _label, bool _verbose = false);

        /** \brief Default destructor
         *
         * Default destructor
		 * 
         */		
        virtual ~NodeBase();

        /** \brief Gets node ID
         *
         * Gets node ID. Inline function.
		 * 
         */
        unsigned int nodeId() const;

        /** \brief Gets node label
         * 
         * Gets node label. Inline function.
		 * 
         */
        std::string nodeLabel() const;

};

inline unsigned int NodeBase::nodeId() const
{
    return node_id_;
}

inline std::string NodeBase::nodeLabel() const
{
    return label_;
}

#endif /* NODE_BASE_H_ */
