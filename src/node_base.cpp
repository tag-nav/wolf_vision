#include "node_base.h"

//init static node counter
unsigned int NodeBase::node_id_count_ = 0;

NodeBase::NodeBase(std::string _label) :
        label_(_label), //
        node_id_(++node_id_count_)
{
    //    std::cout << "NodeID::constructor. Id: " << node_id_ << std::endl;
}

NodeBase::~NodeBase()
{
    //    std::cout << "NodeID::destructor. Id: " << node_id_ << std::endl;
}

