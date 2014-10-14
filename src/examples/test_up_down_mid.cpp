/**
 * \file test_top_down_nodes.cpp
 *
 *  Created on: 06/08/2014
 *     \author: jsola
 */


#include <memory>
#include <list>
#include <iostream>

#include "wolf.h"
#include "node_base.h"

using namespace std;
using namespace Eigen;

class NodeID
{
    protected :
        static unsigned int node_id_count_; ///< object counter acts as simple ID factory
        unsigned int node_id_; ///< node id. It is unique over the whole Wolf Tree

    public :
        NodeID() :
            node_id_(++node_id_count_)
        {
            cout << "NodeID::constructor. Id: " << node_id_ << endl;
        };

        virtual ~NodeID()
        {
            cout << "NodeID::destructor. Id: " << node_id_ << endl;
        };

        unsigned int getId() const
        {
            return node_id_;
        };

        virtual void printLabel(ostream & _ost = cout) const
        {
            _ost <<"NODE ";
        }

        virtual void print(unsigned int _ntabs = 0, ostream & _ost = cout) const
        {
            printLabel(_ost);
            _ost << " " << node_id_ << endl;
        };

};


template<class DownT>
class UpNode
{
    public:
        typedef shared_ptr<DownT> DownNodePtr;
        typedef list<DownNodePtr> DownNodeList;
        typedef typename DownNodeList::iterator DownNodeIter;

    private:
        DownNodeList down_node_list_;

    public:
        UpNode()
        {

        }

        virtual ~UpNode()
        {
            //
        }

        void addDownNode(const DownNodePtr & _ptr)
        {
            down_node_list_.push_back(_ptr);
        }

        DownNodeList & downNodeList()
        {
            return down_node_list_;
        }

};


template<class UpT>
class DownNode
{
    public:
        typedef shared_ptr<UpT> UpNodePtr;
        typedef weak_ptr<UpT> UpNodeWPtr;

    private:
        UpNodeWPtr up_node_wptr_;

    public:
        DownNode(const UpNodePtr & _up_node_ptr)
        {
            linkToUpNode(_up_node_ptr);
        }

        virtual ~DownNode()
        {
            //
        }

        void linkToUpNode(const UpNodePtr & _pptr)
        {
            up_node_wptr_ = _pptr;
        }

        /** \brief Access the shared pointer to parent.
         *
         * Access the shared pointer to parent.
         * Throw if parent has been destroyed.
         *
         */
        UpNodePtr upNodePtr(void)
        {
            UpNodePtr sptr = up_node_wptr_.lock();
            if (!sptr)
            {
                std::cerr << __FILE__ << ":" << __LINE__ << " DownNode::upNodePtr threw weak" << std::endl;
                throw "WEAK";
            }
            return sptr;
        }

};

template<class UpT, class DownT>
class MidNode : public UpNode<DownT>, public DownNode<UpT>
{
    public:
        MidNode(const typename DownNode<UpT>::UpNodePtr & _up_node_ptr) :
            DownNode<UpT>(_up_node_ptr)
    {
            //
    }
};

class CaptureX;

class FrameX : public NodeID, public UpNode<CaptureX>
{
    public:
        typedef shared_ptr<CaptureX> CapturePtr;
        typedef list<CapturePtr> CaptureList;
        typedef CaptureList::iterator CaptureIter;

        FrameX()
        {
            //
        }

        virtual ~FrameX()
        {
            //
        }

        void addCapture(const CapturePtr & _sc_ptr)
        {
            addDownNode(_sc_ptr);
        }

        CaptureList & captureList()
        {
            return downNodeList();
        }

};

class FeatureX;

class CaptureX : public NodeID, public MidNode<FrameX, FeatureX>
{
    public:
        typedef shared_ptr<FrameX> FramePtr;
        typedef shared_ptr<FeatureX> FeaturePtr;
        typedef list<FeaturePtr> FeatureList;
        typedef FeatureList::iterator FeatureIter;

        CaptureX(const FramePtr & _frm_ptr) :
            MidNode(_frm_ptr)
        {
            //
        }

        virtual ~CaptureX()
        {
            //
        }

        void linkToFrame(const FramePtr & _frm_ptr)
        {
            linkToUpNode(_frm_ptr);
        }

        FramePtr framePtr()
        {
            return upNodePtr();
        }

        void addFeature(const FeaturePtr & _f_ptr)
        {
            addDownNode(_f_ptr);
        }

        FeatureList & featureList()
        {
            return downNodeList();
        }


};

class CorrespondenceX;

class FeatureX : public NodeID, public MidNode<CaptureX, CorrespondenceX>
{
    public:
        typedef shared_ptr<CaptureX> CapturePtr;
        typedef shared_ptr<CorrespondenceX> CorrespondencePtr;
        typedef list<CorrespondencePtr> CorrespondenceList;
        typedef CorrespondenceList::iterator CorrespondenceIter;

        FeatureX(const CapturePtr & _sc_ptr) :
            MidNode(_sc_ptr)
        {

        }

        virtual ~FeatureX()
        {
            //
        }

        void linkToCapture(const CapturePtr & _cp_ptr)
        {
            linkToUpNode(_cp_ptr);
        }

        CapturePtr capturePtr()
        {
            return upNodePtr();
        }

        void addCorrespondence(const CorrespondencePtr & _co_ptr)
        {
            addDownNode(_co_ptr);
        }

        CorrespondenceList & correspondenceList()
        {
            return downNodeList();
        }

};

class CorrespondenceX : public NodeID, public DownNode<FeatureX>
{
    public:
        typedef shared_ptr<FeatureX> FeaturePtr;

        CorrespondenceX(const FeaturePtr & _ft_ptr) :
            DownNode(_ft_ptr)
        {

        }

        virtual ~CorrespondenceX()
        {
            //
        }

        void linkToFeature(const FeaturePtr & _ft_ptr)
        {
            linkToUpNode(_ft_ptr);
        }

        FeaturePtr featurePtr()
        {
            return upNodePtr();
        }

};

unsigned int NodeID::node_id_count_ = 0;

int main()
{

    // Build a tree with 1 frame, 2 captures per frame, 2 features per capture, 2 correspondences per feature
    auto framePtr = make_shared<FrameX>();

    for (int cp = 0; cp < 2; cp++)
    {
        framePtr->addCapture(make_shared<CaptureX>(framePtr));
        auto captureIter = framePtr->captureList().rbegin();
        auto capturePtr = *captureIter;

        for (int ft = 0; ft < 2; ft++)
        {
            capturePtr->addFeature(make_shared<FeatureX>(capturePtr));
            auto featureIter = capturePtr->featureList().rbegin();
            auto featurePtr = *featureIter;

            for (int co = 0; co < 2; co++)
            {
                featurePtr->addCorrespondence(make_shared<CorrespondenceX>(featurePtr));
                auto correspondenceIter = featurePtr->correspondenceList().rbegin();
                auto correspondencePtr = *correspondenceIter;

            } // for co

        } // for ft

    } // for cp

    // Traverse the tree, and print IDs
    cout << "\nWOLF TREE" << endl;
    cout << "Frame: " << framePtr->getId() << endl;
    for (auto cp_it = framePtr->captureList().begin(); cp_it != framePtr->captureList().end(); cp_it++)
    {
        cout << "\tCapture: " << (*cp_it)->getId() << endl;

        for (auto ft_it = (*cp_it)->featureList().begin(); ft_it != (*cp_it)->featureList().end(); ft_it++)
        {
            cout << "\t\tFeature: " << (*ft_it)->getId() << endl;

            for (auto co_it = (*ft_it)->correspondenceList().begin(); co_it != (*ft_it)->correspondenceList().end(); co_it++)
            {
                cout << "\t\t\tCorrespondence: " << (*co_it)->getId() << endl;
            }
        }
    }

    return 1;
}


