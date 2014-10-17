#include "node.h"

//init static node counter
unsigned int Node::node_id_count_ = 0;

Node::Node(std::string _label) :
        label_(_label), //
        node_id_(++node_id_count_)
{
    //    std::cout << "NodeID::constructor. Id: " << node_id_ << std::endl;
}

Node::~Node()
{
    //    std::cout << "NodeID::destructor. Id: " << node_id_ << std::endl;
}

