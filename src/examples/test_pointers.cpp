/**
 * \file test_pointers.cpp
 *
 *  Created on: 27/08/2014
 *     \author: jsola
 */

#include <memory>
#include <iostream>
using namespace std;

class DataAndPointer
{
    public:
        std::string label_;
        DataAndPointer* ptr_;

        DataAndPointer(string _label) : label_(_label), ptr_(NULL)
        {
            cout << label_ << " allocated." << endl;
        }
        virtual ~DataAndPointer()
        {
            cout << label_ << " freed." << endl;
        }
        void print()
        {
            cout << "I am  " << label_ << endl;
        }
        void printPointed()
        {
            if (ptr_ == NULL)
                throw("Pointer is NULL.");
            else
                cout << "She is  " << ptr_->label_ << endl;
        }
        void printSelfAddr()
        {
            cout << label_ << " is at: " << this << endl;
        }
        void printPointedAddr()
        {
            cout << label_ << " points to: " << ptr_ << endl;
        }
};

/**
 * We test whether pointers to freed data notice that the data has been freed.
 *
 * CONCLUSION: Pointers to freed data do not notice the pointed memory has been freed.
 *
 * This is the output of this main(), with 2 comments.
 *   - A allocated.
 *   - B allocated.
 *   - A is at: 0x7fff68238bf0
 *   - B is at: 0x7fff68238c00
 *   - A points to: 0x7fff68238c00
 *   - B freed.
 *   - I am  A
 *   - A points to: 0x7fff68238c00      <-- A still points to B, but we just freed B !
 *   - She is  B                        <-- Here we still see B, but B is freed !
 *   - C allocated.
 *   - She is  C                        <-- Here we see C, which luckily took the place of B !
 *   - C freed.
 *   - A freed.
 */
int main()
{

    DataAndPointer a("A");
    {
        DataAndPointer b("B");
        a.ptr_ = &b;
        a.printSelfAddr();
        b.printSelfAddr();
        a.printPointedAddr();
    } // Force free of b.
    a.print();
    a.printPointedAddr();
    a.printPointed();
    DataAndPointer c("C");
    a.printPointed();
}
