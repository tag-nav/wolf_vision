/*
 * \file test_parent_child.cpp
 *
 *  Created on: Jun 19, 2014
 *      \author: jsola
 */


#include <memory>

#include "wolf.h"
#include "parent_of.h"
#include "child_of.h"

using namespace std;

class Baby;

class Papa : public ParentOf<int,Baby>
{
    public:
    virtual ~Papa(){};
    string name(){return "papa";}
    void display();
};
typedef shared_ptr<Papa> PapaPtr;

class Baby : public ChildOf<Papa>
{
    public:
        Baby(const PapaPtr & _pptr) :
            ChildOf<Papa>(_pptr)
        {
            
        };
        
        virtual ~Baby()
        {
            
        };
        
        string name()
        {
            return "baby";
        }

        void display();
};
typedef shared_ptr<Baby> BabyPtr;

void Papa::display()
{
    cout << "papa of ";
    for (auto it = child_map_.begin(); it != child_map_.end(); it++)
    {
        cout << it->second->name() << ", ";
    }
    cout << endl;
}
void Baby::display()
{
    cout << "baby of ";
    cout << parentPtr()->name() << endl;
}


int main()
{
    cout << "\nChildOf and ParentOf -- demo";
    cout << "\n============================" << endl;

    cout << "\n1. creating and linking, and printing relations..." << endl;
    PapaPtr papa_ptr(new Papa);
    //BabyPtr baby_ptr(new Baby);
    // link them together
    auto it = papa_ptr->registerChild(1, BabyPtr(new Baby(papa_ptr)) );
//     baby_ptr->linkToParent(papa_ptr);
    //it->second->linkToParent(papa_ptr);
    cout << "baby_ptr.use_count()= " << it->second.use_count() << endl;

    // print info needing to access child from parent
    papa_ptr->display();
    // print info needing to access parent from child
    it->second->display();

//     cout << "\n2. unlinking from parent... \t'baby_ptr->unlinkFromParent()'" << endl;
//     baby_ptr->unlinkFromParent();
//    baby_ptr->display(); //  trows!
//     cout << "... and relinking for later use" << endl;
//     baby_ptr->linkToParent(papa_ptr);

    cout << "\n3. unregistering baby... \t'papa_ptr->unregisterChild(papa_ptr->child_map_.begin())'" << endl;
    papa_ptr->unregisterChild(it);
//     cout << "... and re-registering for later use" << endl;
//     papa_ptr->registerChild(1, baby_ptr);
//     baby_ptr->linkToParent(papa_ptr);

    {
        cout << "\n4. just creating and destroying, no linking" << endl;
        // Create a papa and a baby
        PapaPtr papa_ptr2(new Papa);
        BabyPtr baby_ptr2(new Baby);
    }  // they die here because of out-of-scope!

//     cout << "\nCONCLUSION: It works fine but a dying child should unregister from parent before dying.\n";
//     cout << "Actually resetting its own pointer seems idiotic since he's dying anyway.\n";
//     cout << "The thing is that the pointer in parent does not know that the child died, and therefore "
//             "the childs needs to ask his parent to unregister him beforehand.\n";
//     cout << "This does not happen the other way around, when parents die, as parents have shared_ptr "
//             "to children and, one unregirstered, children are allowed to die." << endl;
//     cout << "See in 5. below what happens when we go out of scope:" << endl;

    cout << "\n5. exiting main()..." << endl;
    return 0;
}
