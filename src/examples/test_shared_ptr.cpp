/*
 * test_graph_node_base.cpp
 *
 *  Created on: Jun 19, 2014
 *      \author: andreu
 */

#include <memory>
#include <iostream>


using namespace std;

class FooFoo
{
	public:
	FooFoo()
	{
		cout << "FooFoo created\n";
	}
	~FooFoo()
	{
		cout << "FooFoo destroyed\n";
	}

	void sayFoo()
	{
		cout << "Fooooooooooooooooooofooooooooooooo!\n";
	}
};

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
        
        virtual ~Foo()
        { 
            cout << "~Foo() - Destructor - " << id_ << endl; 
        }

        virtual void doSomething()
        {
            cout << "Processing - " << id_ << endl;
        }
};

class Bar : public Foo
{
    protected:
        unsigned int data_;
    public:
        Bar() : Foo(), data_(0)
        {
            //
        }
        Bar(unsigned int _data) : Foo(), data_(_data)
        {
            //
        }
        virtual ~Bar()
        {
            //
        }
        
        virtual void doSomething()
        {
            cout << "Processing - " << id_ << ": " << data_ << endl;
        }        
};

//deleter 
struct Deleter
{ 
    void operator()(Foo* p) const
    {
        cout << "Call deleter for Foo object...\n";
        delete p;
    }
};

void myFunction(const std::shared_ptr<Foo> & _foo_shptr)
{
    _foo_shptr->doSomething();
}


//init static variables
unsigned int Foo::count_ = 10;
 
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
    
    cout << "\n***** Checking polimorphism\n";
    shared_ptr<Bar> bar_shptr_(new Bar(23));
    //myFunction(sh5);
    myFunction(bar_shptr_);
    
    cout << "\n***** Checking the destruction outside a scope\n";
    shared_ptr<FooFoo> sh6;
    if (1)
    {
    	FooFoo myFooFoo;
    	sh6.reset(&myFooFoo);
    	cout << "inside:\n";
    	sh6->sayFoo();
    }
	cout << "outside:\n";
	sh6->sayFoo();


    cout << "\n***** End of test. Still three shared_ptr's owning objects. They will be deleted at the end of main\n";    
    return 0;
}
