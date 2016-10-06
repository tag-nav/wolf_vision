/**
 * \file test_sh_ptr.cpp
 *
 *  Created on: Oct 4, 2016
 *      \author: jsola
 */


#include "wolf.h"

#include <memory>

using std::shared_ptr;
using std::weak_ptr;
using std::enable_shared_from_this;
using std::list;

namespace wolf
{

class H;
class S;
class p;
class T;
class F;
class C;
class f;
class c;
class M;
class L;

class P : public enable_shared_from_this<P>
{
        H H_;
        T T_;
        M M_;
    public:
        P();
        ~P(){}
};

class H : public enable_shared_from_this<H>
{
        weak_ptr<P> P_ptr_;
        list<shared_ptr<S>> S_list_;
    public:
        H(){}
        ~H(){}
        shared_ptr<P> getP(){
            shared_ptr<P> P_sh = P_ptr_.lock();
            // if (P_sh) // check will be made by caller after all
            return P_sh;
        }
        void setP(shared_ptr<P> _P){P_ptr_ = _P;}
        list<shared_ptr<S>>& getSlist() {return S_list_;}
        void addS(shared_ptr<S> _S);
};

class S : public enable_shared_from_this<S>
{
        weak_ptr<P> P_ptr_;
        weak_ptr<H> H_ptr_;
        list<shared_ptr<p>> p_list_;
    public:
        S(){}
        ~S(){}
        shared_ptr<P> getP(){
            shared_ptr<P> P_sh = P_ptr_.lock();
            // if (P_sh) // check will be made by caller after all
            return P_sh;
        }
        void setP(shared_ptr<P>& _P){P_ptr_ = _P;}
        shared_ptr<H> getH(){
            shared_ptr<H> H_sh = H_ptr_.lock();
            // if (h_sh) // check will be made by caller after all
            return H_sh;
        }
        void setH(shared_ptr<H> _H){H_ptr_ = _H;}
        list<shared_ptr<p>>& getplist() {return p_list_;}
        void addp(shared_ptr<p> _p);
};

class p : public enable_shared_from_this<p>
{
        weak_ptr<P> P_ptr_;
        weak_ptr<S> S_ptr_;
    public:
        p(){}
        ~p(){}
        shared_ptr<P> getP(){
            shared_ptr<P> P_sh = P_ptr_.lock();
            // if (p_sh) // check will be made by caller after all
            return P_sh;
        }
        void setP(shared_ptr<P>& _P){P_ptr_ = _P;}
        shared_ptr<S> getS(){
            shared_ptr<S> S_sh = S_ptr_.lock();
            // if (h_sh) // check will be made by caller after all
            return S_sh;
        }
        void setS(shared_ptr<S> _S){S_ptr_ = _S;}
};

//////////////////////////////////////////

void P::P(){
    H_.setP(shared_from_this());
    //    T_.setP(shared_from_this()); // uncomment after implementing T
    //    M_.setP(shared_from_this()); // uncomment after implementing M
}

void H::addS(shared_ptr<S> _S)
{
    S_list_.push_back(_S);
    _S->setH(shared_from_this());
}

void S::addp(shared_ptr<p> _p)
{
    p_list_.push_back(_p);
    _p->setS(shared_from_this());
}

}
