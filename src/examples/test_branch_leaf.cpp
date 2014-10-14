/*
 * \file test_parent_child.cpp
 *
 *  Created on: Jun 19, 2014
 *      \author: jsola
 */

#include "wolf.h"
#include "branch_node_of.h"
#include "leaf_node_of.h"

using namespace std;
using namespace Eigen;

class Baby;

class Papa : public BranchNodeOf<int, Baby>
{
    protected:
        string name_;

    public:
        Papa(const string _name) :
                name_(_name)
        {
            //
        }

        virtual ~Papa()
        {
        }

        string & name()
        {
            return name_;
        }
        void display();

};
typedef shared_ptr<Papa> PapaPtr;

class Baby : public LeafNodeOf<Papa>
{
    protected:
        string name_;
        static const unsigned int ERROR_SIZE_ = 2;

    public:
        Baby(const PapaPtr & _pptr, string _name) :
                LeafNodeOf<Papa>(_pptr, ERROR_SIZE_), //
                name_(_name)    //
        {
            //
        }

        virtual ~Baby()
        {
            //
        }

        string & name()
        {
            return name_;
        }

        void display()
        {
            cout << "I am " << name() << ", baby of ";
            cout << parentPtr()->name() << endl;
        }

        virtual VectorXs computeExpectationError()
        {
            expectation_error_ =  VectorXs::Constant(2, 2); // Just an example of a last child computing its own error.
            expectation_error_map_ = expectation_error_;
            return expectation_error_;
        }

};

typedef shared_ptr<Baby> BabyPtr;

void Papa::display()
{
    cout << "I am " << name() << ", papa of ";
    for (auto it = child_map_.begin(); it != child_map_.end(); it++)
    {
        cout << it->second->name() << ", ";
    }
    cout << endl;
}

/**********************************
 *         MAIN
 **********************************/

int main()
{
    cout << "\nBranches and leaves -- demo";
    cout << "\n===========================" << endl;

    cout << "\n-- Creating and linking, and printing relations..." << endl;
    PapaPtr papa_ptr(new Papa("Jacob"));
    // Bear children and link to them in one step...
    auto it = papa_ptr->registerChild(1, BabyPtr(new Baby(papa_ptr,"Andy")));
    it = papa_ptr->registerChild(1, BabyPtr(new Baby(papa_ptr,"Beny")));
    it = papa_ptr->registerChild(1, BabyPtr(new Baby(papa_ptr,"Cody")));

    // print info needing to access child from parent
    papa_ptr->display();
    // print info needing to access parent from child
    it->second->display();
    cout << "papa_ptr->childMap().size() = " << papa_ptr->childMap().size() << endl;
    cout << "it->second.use_count() = " << it->second.use_count() << endl;

    cout << "\n-- Computing all expectation errors..." << endl;
    unsigned int idx = 0;
    VectorXs expectation_error_storage_(papa_ptr->expectationErrorSize());
    papa_ptr->mapExpectationErrors(expectation_error_storage_, idx);
    papa_ptr->computeExpectationError();
    cout << "expectation error storage = " << expectation_error_storage_.transpose() << endl;

    cout << "\n-- Unregistering baby... \t'papa_ptr->unregisterChild(it)'" << endl;
    papa_ptr->unregisterChild(it);
    cout << "papa_ptr->childMap().size() = " << papa_ptr->childMap().size() << endl;
    cout << "it->second.use_count()= " << it->second.use_count() << endl;

    cout << "\n-- Computing all expectation errors..." << endl;
    idx = 0;
    expectation_error_storage_.resize(papa_ptr->computeExpectationErrorSize());
    papa_ptr->mapExpectationErrors(expectation_error_storage_, idx);
    papa_ptr->computeExpectationError();
    cout << "expectation error storage = " << expectation_error_storage_.transpose() << endl;

    {
        cout << "\n-- Just creating and destroying, no registering." << endl;
        // Create a papa and a baby
        PapaPtr papa_ptr2(new Papa("James"));
        Baby Amy(papa_ptr2,"Amy");
        BabyPtr AliPtr(new Baby(papa_ptr2,"Ali"));
    }  // they die here because of out-of-scope!

    cout << "\nCONCLUSION: "
            "It works fine: "
            "\n\tChilds are born by registering new instances into the parent's multimap, with:"
            "\n\t\tauto it = papa_ptr->registerChild(1, BabyPtr( new Baby(papa_ptr) ) )"
            "\n\tUnregistering a child from a parent automatically deletes the child, with"
            "\n\t\tpapa_ptr->unregisterChild(it)"
            "\n\tExiting from context conveniently destroys all objects."
            "\n\tInheriting from ParentOfExpecter correctly maps and computes expectation errors of children down to the leaves." << endl;

    cout << "\n-- Exiting main()..." << endl;
    return 0;
}
