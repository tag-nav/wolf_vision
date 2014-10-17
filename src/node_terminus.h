/**
 * \file node_terminus.h
 *
 *  Created on: 12/08/2014
 *     \author: jsola
 */

#ifndef NODE_TERMINUS_H_
#define NODE_TERMINUS_H_

//wolf includes
#include "node_constrainer.h"

/**
 * \brief Dummy node to terminate the tree in both the Top and Bottom levels.
 *
 * This node does nothing. It only shares basic types with other nodes
 * to be able to be passed to Top and Bottom nodes as
 * their template parameters.
 * 
 * Top node will usually be Frame, while Bottom should be where error function is implemented. 
 * Depending on the applcation it ca be at Correspondence level, but also on feature of capture levels.
 *
 * Use it as template parameter for TOP and BOTTOM nodes.
 *
 */
class NodeTerminus : public NodeConstrainer<NodeTerminus, NodeTerminus>
{
    protected:
        virtual ~NodeTerminus();
};

////////////////////////////////
// IMPLEMENTATION
////////////////////////////////



inline NodeTerminus::~NodeTerminus()
{
  //
}

#endif /* NODE_TERMINUS_H_ */
