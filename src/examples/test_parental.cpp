/*
 * test_parental.cpp
 *
 *  Created on: Jun 16, 2014
 *      \author: jsola
 */

#include "parental.h"


int main()
{

    cout << "\nParents and Childs -- demo";
    cout << "\n==========================\n" << endl;

    cout << "\n1. Creating parents..." << endl;
    ParentPtr jacobPtr(new Father("Jacob"));
    ParentPtr jamesPtr(new Father("James"));

    cout << "\n2. Creating children and linking them to parents - upward only" << endl;
    ChildPtr alexPtr(new Boy(jacobPtr, "Alex"));
    ChildPtr andyPtr(new Boy(jacobPtr, "Andy"));

    ChildPtr arnyPtr(new Boy(jamesPtr, "Arny"));
    ChildPtr aidaPtr(new Girl(jamesPtr, "Aida"));

    cout << "arnyPtr.use_count(): " << arnyPtr.use_count() << endl;
    cout << endl;

    cout << "\n3. Linking parents to childs -- downward" << endl;
    cout << "doing e.g. 'jamesPtr->newChild(arnyPtr)' for everyone" << endl;
    jamesPtr->newChild(arnyPtr);
    jacobPtr->newChild(alexPtr);
    jacobPtr->newChild(andyPtr);
    jamesPtr->newChild(aidaPtr);
    cout << "arnyPtr.use_count(): " << arnyPtr.use_count() << endl;
    cout << endl;

    cout << "\n4. Calling child processes from the base children" << endl;
    cout << "doing e.g. 'alexPtr->process()' for everyone" << endl;
    alexPtr->process();
    andyPtr->process();
    arnyPtr->process();
    aidaPtr->process();
    cout << endl;

    cout << "\n5. Calling parent processes from the base parents" << endl;
    cout << "doing e.g. 'jacobPtr->process()' for everyone" << endl;
    jacobPtr->process();
    cout << endl;
    jamesPtr->process();
    cout << endl;
   
    cout << "doing 'arnyPtr.reset()'" << endl;
    arnyPtr.reset();
    cout << "arnyPtr.use_count(): " << arnyPtr.use_count() << endl;
    if (arnyPtr) cout << "arnyPtr is set" << endl;
    else cout << "arnyPtr is NULL\n";
    cout << endl;
    
    //delete jacobPtr;
    return 0;
}
