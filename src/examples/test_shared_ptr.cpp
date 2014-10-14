/*
 * test_graph_node_base.cpp
 *
 *  Created on: Jun 19, 2014
 *      \author: andreu
 */

#include <memory>
#include <iostream>


using namespace std;

//Object Foo class
class Foo
{
    protected:
        static unsigned int count_;
        unsigned int id_; 
        
    public:
        Foo() 
        { 
            id_ = count_; 
            count_++; 
            cout << "Foo() - Constructor - " << id_ << endl; 
        }
        
        ~Foo()
        { 
            cout << "~Foo() - Destructor - " << id_ << endl; 
        }

        void doSomething()
        {
            cout << "Processing - " << id_ << endl;
        }
};
unsigned int Foo::count_ = 10;

//deleter 
struct Deleter
{ 
    void operator()(Foo* p) const
    {
        cout << "Call deleter for Foo object...\n";
        delete p;
    }
};
 
//main
int main()
{
    cout << "\n***** Constructor with no managed object\n";
    shared_ptr<Foo> sh1;
    cout << sh1.use_count() << '\n';
    cout << sh1.unique() << '\n';

    cout << "\n***** Constructor with object\n";
    shared_ptr<Foo> sh2(new Foo);
    sh2->doSomething();
    cout << sh2.use_count() << "," << sh2.unique() << '\n';
    
    cout << "\n***** Constructor with shared_ptr\n";
    shared_ptr<Foo> sh3(sh2);
    sh3->doSomething();
    cout << sh2.use_count() << "," << sh2.unique() << '\n';
    cout << sh3.use_count() << "," << sh3.unique() << '\n';

    cout << "\n***** Constructor with object and deleter\n";
    shared_ptr<Foo> sh4(new Foo, Deleter());

    cout << "\n***** Reset of shared_ptr created with specific Deleter\n";    
    sh4.reset();

    cout << "\n***** Reset of sh2\n";    
    sh2.reset();
    cout << sh2.use_count() << "," << sh2.unique() << '\n';
    cout << sh3.use_count() << "," << sh3.unique() << '\n';
    
    cout << "\n***** Constructor with an already created object\n";
    Foo * myFoo = new Foo();
    shared_ptr<Foo> sh5(myFoo);
    cout << sh5.use_count() << "," << sh5.unique() << '\n';
    //sh5.reset();
    
    cout << "\n***** End of test. Still two shared_ptr's owning objects. They will be deleted at the end of main\n";    
    return 0;
}