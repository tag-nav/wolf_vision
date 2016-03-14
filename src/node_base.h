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
        static unsigned int node_id_count_; ///< Object counter (acts as simple ID factory)

        std::string label_; ///< Text label identifying the node
        unsigned int node_id_; ///< Node id. It is unique over the whole Wolf Tree
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

#include <iostream>

inline NodeBase::NodeBase(std::string _label, bool _verbose) :
        label_(_label), node_id_(++node_id_count_), verbose_(_verbose)
{
    if (verbose_)
        std::cout << "NodeBase::NodeBase(). Id: " << node_id_ << " Label: " << label_ << std::endl;
}

inline NodeBase::~NodeBase()
{
    //
}

inline unsigned int NodeBase::nodeId() const
{
    return node_id_;
}

inline std::string NodeBase::nodeLabel() const
{
    return label_;
}

#endif /* NODE_BASE_H_ */
