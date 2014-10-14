/**
 * \file test_wolf_tree.cpp
 *
 *  Created on: 04/08/2014
 *     \author: jsola
 */

#include <memory>
#include <typeinfo>

#include "state_pqv.h"
#include "frame.h"
#include "sensor_capture_image.h"
#include "feature_image_point.h"
#include "correspondence_image_point.h"

//namespaces
using namespace std;
using namespace Eigen;

//id count init
//template<>
//unsigned int Frame<StatePQV>::frame_id_count_ = 0;
unsigned int Frame::frame_id_count_ = 0;
unsigned int NodeBase::node_id_count_ = 0;

int main(){

    cout << "\n Test Wolf tree";
    cout << "\n================\n" << endl;

    VectorXs state_storage(10); // will be resized
    VectorXs error_storage(10); // will be resized

    shared_ptr<StatePose> state_ptr(new StatePQV);

//    shared_ptr<Frame<StatePQV> > frame1(new Frame<StatePQV>(KEY_FRAME, 0, state_ptr)), frame2(new Frame<StatePQV>(KEY_FRAME, 0, state_ptr));
    shared_ptr<Frame> frame1(new Frame(KEY_FRAME, 0, state_ptr)), frame2(new Frame(KEY_FRAME, 0, state_ptr));

    shared_ptr<SensorCaptureImage> capture1(new SensorCaptureImage(frame1)), capture2(new SensorCaptureImage(frame1));

    shared_ptr<FeatureImagePoint> feat1(new FeatureImagePoint(capture1)), feat2(new FeatureImagePoint(capture1)), feat3(new FeatureImagePoint(capture2)), feat4(new FeatureImagePoint(capture2));

    shared_ptr<CorrespondenceImagePoint>
            cor1(new CorrespondenceImagePoint), cor2(new CorrespondenceImagePoint),
            cor3(new CorrespondenceImagePoint), cor4(new CorrespondenceImagePoint),
            cor5(new CorrespondenceImagePoint), cor6(new CorrespondenceImagePoint),
            cor7(new CorrespondenceImagePoint), cor8(new CorrespondenceImagePoint);

    feat1->registerDownNode(shared_ptr<NodeLinked>(new CorrespondenceImagePoint(dynamic_pointer_cast<NodeLinked>(feat1))));

    feat1->addDownNode(cor1);
    feat1->addDownNode(cor2);
    feat2->addDownNode(cor3);
    feat2->addDownNode(cor4);
    feat3->addDownNode(cor5);
    feat3->addDownNode(cor6);
    feat4->addDownNode(cor7);
    feat4->addDownNode(cor8);

    capture1->addDownNode(feat1);
    capture1->addDownNode(feat2);
    capture2->addDownNode(feat3);
    capture2->addDownNode(feat4);

    frame1->addDownNode(capture1);
    frame1->addDownNode(capture2);

    // Print structure
    frame1->print();

    // remap error vector (compute dim, resize, and remap)
    frame1->computeDimError();
    error_storage.resize(frame1->getDimError());
    frame1->remap(error_storage, 0);

    // compute error
    frame1->error();

    cout << "Errors are naively set to coincide with the correspondence's ID. Then, \n";
    cout << "   error_storage = " << error_storage.transpose() << endl;
    cout << "   error_Frm  1  = " << frame1->getError().transpose() << endl;
    cout << "   error_Capt 2  = " << capture2->getError().transpose() << endl;
    cout << "   error_feat 3  = " << feat3->getError().transpose() << endl;
    cout << "   error_corr 4  = " << cor4->getError().transpose() << endl;

    // Access derived pointers
    cout << "Access derived pointers..." << endl;
    // 1. Get the concerned correspondence
    auto cor_iter = feat1->downNodeList().begin(); // The first one is the one we fully linked at derived level.
    // 2. Cast it to the derived type
    shared_ptr<CorrespondenceImagePoint> cip = static_pointer_cast<CorrespondenceImagePoint>(*cor_iter);
    // 3. Access relevant info up the tree
    FeatureBase fi = cip->feature();
    SensorCaptureBase sci = cip->sensorCapture();
    Frame Fi = cip->frame();

    cout << "   VAR    =       typeid(var)).name()         \t\t -     ADDRESS" << endl;
    cout << "---------        ---------------------        \t\t    --------------" << endl;
    cout << "feat1     = " << typeid(feat1).name() <<       "\t\t -  " << feat1       << endl;
    cout << "cor_iter  = " << typeid(cor_iter).name() <<     " - Cannot print " << endl; //cor_iter; cout   << endl; // Note: cannot print iterator address.
    cout << "*cor_iter = " << typeid(*cor_iter).name() <<   "\t\t -  " << *cor_iter   << endl;
    cout << "cip       = " << typeid(cip).name() <<           "\t -  " << cip         << endl;
    cout << "fi        = " << typeid(fi).name() <<      "\t\t\t\t -  " << &fi         << endl;
    cout << "sci       = " << typeid(sci).name() <<     "\t\t\t\t -  " << &sci        << endl;
    cout << "Fi        = " << typeid(Fi).name() <<    "\t\t\t\t\t -  " << &Fi         << endl;
    cout << endl;

    cout << "feat1->measurement().size() = " << feat1->measurement().size(); cout << endl;
    cout << "fi.measurement().size() = " << fi.measurement().size(); cout << endl;
    cout << "Fi.state().q() = " << Fi.state().q().coeffs().transpose(); cout << endl;
//    cout << "scip.lock()->raw_ptr_->size() = " ; scip.lock()->raw_ptr_->rawData().size();

    return 0;
}

