
#include "graph_node_base.h"

GraphNodeBase::GraphNodeBase(const shared_ptr<GraphNodeBase>& _parent_node, const int _id) :
        parent_(_parent_node), 
        forward_correspondence_(nullptr), 
        backward_correspondence_(nullptr),
        id_(_id)
{
    //
}

GraphNodeBase::~GraphNodeBase()
{
    //
}

void GraphNodeBase::setForwardCorrespondence(GraphNodeBase & _fwd_c)
{
    forward_correspondence_.reset( &_fwd_c );
}

void GraphNodeBase::setBackwardCorrespondence(GraphNodeBase & _bckwd_c)
{
    backward_correspondence_.reset( &_bckwd_c );
}

void GraphNodeBase::print()
{
    cout << id_ << endl;
}
