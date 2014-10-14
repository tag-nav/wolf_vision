/*
 * circular_buffer.h
 *
 *  Created on: May 16, 2014
 *      \author: jsola
 */

#ifndef CIRCULAR_BUFFER_H_
#define CIRCULAR_BUFFER_H_

#include <iostream>
#include <vector>

using namespace std;

/** \brief class for circular buffer
 *
 * Uses exact storage of _capacity nodes
 *
 * Head is pointed by _head
 *
 * Tail is pointed by (_head + 1 + _capacity - _size) % _capacity
 *
 * Size is controlled by _size,
 *
 * Maximum size is set by _capacity
 *
 * No methods for resizing, reserve, etc, are available
 * \param Node the type of node contained in the buffer
 */
template<class Node>
class CircularBuffer
{
    protected:
        unsigned int capacity_;         ///< Maximum length
        unsigned int head_;             ///< index of head in non-circular container
        unsigned int size_;             ///< current length
        vector<Node> array_;            ///< buffer non-circular container

    public:

        /**
         * Constructor from size
         * \param _size maximum length of buffer
         */
        CircularBuffer(unsigned int _size);

        /**
         * Constant constructor from size and one node.
         * \param _size maximum length of buffer
         * \param n the node to insert in all buffer positions
         */
        CircularBuffer(unsigned int _size, const Node& n);

        /**
         * get buffer size
         */
        unsigned int size();

        /**
         * check for full buffer
         */
        bool full();

        /**
         * check for empty buffer
         */
        bool empty();

        /**
         * index of head in non-circular storage
         */
        unsigned int head();

        /**
         * index of tail in non-circular storage
         */
        unsigned int tail();

        /**
         * maximum length of buffer
         */
        unsigned int capacity();

        /** add node to head
         * \param _node the node to add
         */
        void push_head(Node& _node);

        /**
         * Remove node from tail
         *
         * --- Not to be used in CargoANTs
         */
        Node pop_tail();

        /**
         * get a node, count from head
         *
         * from_head(0) = array(head), newest node
         *
         * \param _index index to count from head
         */
        Node& from_head(unsigned int _index);

        /**
         * get a node, cunt from tail
         *
         * from_tail(0) = array(tail()), oldest node
         *
         * \param _index index to count from tail
         */
        Node& from_tail(unsigned int _index);

};

template<class Node>
ostream& operator<<(ostream & os, CircularBuffer<Node>& cb)
{
    os << "H:" << cb.head() << "; T:" << cb.tail() << "; S:" << cb.size() << "; F:" << cb.full() << "; E:" << cb.empty()
            << ";    ";
    os << " >-";
    unsigned int i = 0;
    while (i < cb.size())
    {
        os << cb.from_tail(i++) << "-";
    }
    os << ">";
    return os;
}

template<class Node>
CircularBuffer<Node>::CircularBuffer(unsigned int _size, const Node& n) :
        capacity_(_size), head_(_size), size_(0), array_(_size, n)
{
    for (unsigned int i = 0; i < _size; i++)
    {
        //      new(&_array.at(i)) Node(&(_array(0).x(0)) + i*Node::SIZE_, Node::SIZE_);
    }
}

template<class Node>
CircularBuffer<Node>::CircularBuffer(unsigned int _size) :
        capacity_(_size), head_(_size), size_(0), array_(_size)
{
}

template<class Node>
unsigned int CircularBuffer<Node>::size()
{
    return size_;
}

template<class Node>
bool CircularBuffer<Node>::full()
{
    return (size_ == capacity_);
}

template<class Node>
bool CircularBuffer<Node>::empty()
{
    return (size_ == 0);
}

template<class Node>
unsigned int CircularBuffer<Node>::head()
{
    return head_;
}

template<class Node>
unsigned int CircularBuffer<Node>::tail()
{
    return (head_ - size_ + 1 + capacity_) % capacity_;
}

template<class Node>
unsigned int CircularBuffer<Node>::capacity()
{
    return capacity_;
}

template<class Node>
void CircularBuffer<Node>::push_head(Node& node)
{
    if (!full())
    {
        // increase size
        size_++;
    }
    head_ = (head_ + 1) % capacity_;
    array_.at(head_) = node; // Overloading Node::operator= can make this operation keep storage mapping in Nodes
}

template<class Node>
Node CircularBuffer<Node>::pop_tail()
{
    if (empty())
    {
        return -1;
    }
    else
    {
        int tmp = tail();
        size_--;
        return array_.at(tmp);
    }
}

template<class Node>
Node& CircularBuffer<Node>::from_head(unsigned int index)
{
    int it = (head() - index + capacity_) % capacity_;
    return array_.at(it);
}

template<class Node>
Node& CircularBuffer<Node>::from_tail(unsigned int index)
{
    int it = (tail() + index) % capacity_;
    return array_.at(it);
}

#endif /* CIRCULAR_BUFFER_H_ */
