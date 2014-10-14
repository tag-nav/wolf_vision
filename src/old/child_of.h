/**
 * \file child_of.h
 *
 *  Created on: 17/06/2014
 *  \author: jsola
 */

#ifndef CHILD_OF_H_
#define CHILD_OF_H_

//std
#include <iostream>
#include <memory>

using namespace std;

/** \brief Child relation.
 *
 * Inherit from this class to get a Child relation to an object of another class.
 * The relation is through a weak pointer to the parent object (a weak_ptr) of a base type.
 *
 * The parent class can be known only partially:
 * (i.e. class Parent; class Child : public ChildOf<Parent> {...}; class Parent { ... };)
 *
 * \param Parent the base class of the parent.
 *
 * See also ParentOf
 * 
 */
template<class Parent>
class ChildOf
{
    public:
        typedef weak_ptr<Parent> ParentWeakPtr; ///< Weak pointer to parent, type
        typedef shared_ptr<Parent> ParentPtr; ///< Shared pointer to parent, type

    private:
        ParentWeakPtr parent_wptr_;///< Weak pointer to parent.

    public:
        ChildOf()
        {
            //
        }
        
        /** \brief Constructor with arguments 
         * 
         * Construct and link to parent
         * \param _pptr pointer to parent
         * 
         */
        ChildOf(const ParentPtr & _pptr)
        {
            linkToParent(_pptr);
            // cout << "ChildOf::created" << endl;
        }        
        
        /** \brief Destructor.
         * 
         * Destructor. Do nothing. 
         * 
         */
        ~ChildOf(void)
        {
            // cout << "ChildOf::dying" << endl;
        }

        /** \brief Link to parent.
         * 
         * Link to parent.
         * @param _pptr pointer to parent
         * 
         */
        void linkToParent(const shared_ptr<Parent> & _pptr)
        {
            // cout << "ChildOf::setting parent_wptr_" << endl;
            parent_wptr_ = _pptr;
        }

        /** \brief Remove the link to the Parent. */
        void unlinkFromParent(void)
        {
            // cout << "ChildOf::resetting parent_wptr_" << endl;
            parent_wptr_.reset();
        }

        /** \brief Access the shared pointer to parent. 
         * 
         * Access the shared pointer to parent. 
         * Throw if parent has been destroyed. 
         *          
         */
        ParentPtr parentPtr(void)
        {
            ParentPtr sptr = parent_wptr_.lock();
            if (!sptr)
            {
                std::cerr << __FILE__ << ":" << __LINE__ << " ChildOf::parentPtr threw weak" << std::endl;
                throw "WEAK";
            }
            return sptr;
        }
        
        /** \brief Access the shared pointer to parent.
         * 
         * Access the shared pointer to parent. 
         * Throw if parent has been destroyed. 
         * 
         */
        const ParentPtr parentPtr(void) const
        {
            ParentPtr sptr = parent_wptr_.lock();
            if (!sptr)
            {
                std::cerr << __FILE__ << ":" << __LINE__ << " ChildOf::parentPtr const threw weak" << std::endl;
                throw "WEAK";
            }
            return sptr;
        }
        
        /** \brief Access a reference to parent. 
         * 
         * Access a reference to parent. 
         * Throw if parent has been destroyed. 
         *          
         */
        Parent& parent(void)
        {
            ParentPtr sptr = parent_wptr_.lock();
            if (!sptr)
            {
                std::cerr << __FILE__ << ":" << __LINE__ << " ChildOf::parent threw weak" << std::endl;
                throw "WEAK";
            }
            return *sptr;
        }
        
        /** \brief Access a reference to parent.
         * 
         * Access a reference to parent. 
         * Throw if parent has been destroyed. 
         * 
         */
        const Parent& parent(void) const
        {
            ParentPtr sptr = parent_wptr_.lock();
            if (!sptr)
            {
                std::cerr << __FILE__ << ":" << __LINE__ << " ChildOf::parent const threw weak" << std::endl;
                throw "WEAK";
            }
            return *sptr;
        }
};
#endif /* CHILD_OF_H_ */
