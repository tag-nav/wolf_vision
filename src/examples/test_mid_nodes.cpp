/**
 * \file test_top_down_nodes.cpp
 *
 *  Created on: 06/08/2014
 *     \author: jsola
 */

#include "vehicle_base.h"
#include "frame.h"
#include "capture_image.h"
#include "image_point.h"
#include "epipolar_constraint.h"

#include "state_pose.h"
#include "raw_base.h"
#include "pin_hole.h"
#include "trans_pin_hole.h"

#include <vector>
#include <map>
#include <memory>

using namespace std;
using namespace Eigen;

unsigned int Node::node_id_count_ = 0;

int main(int argc, char *argv[])
{


    // 1.  Parse input parameters
    cout << "\nTemplated constrainer nodes demo - test_mid_nodes.cpp.";
    cout << "\n======================================================\n" << endl;
    unsigned int N_iterations;
    // decode program arguments
    if(argc < 2) {
        N_iterations = 1;
        cout << "Usage: test_mid_nodes [N] : performs N iterations of error computation.\nNo argument given: defaults to N = ";
    }
    else
    {
        N_iterations = strtoul(argv[1], NULL, 10);
    }
    cout << N_iterations << ( N_iterations == 1 ? " iteration." : " iterations. " ) << endl << endl;


    // 2. Initialize data

    // Error vector
    VectorXs error_storage;
    Map<VectorXs> error_storage_map(error_storage.data(), 0);

    // Vehicle pose
    Vector3s p(3, 2, 1);
    Quaternions q(1, 2, 3, 4); // Eigen sucks with this unordered quaternion initialization!
    q.normalize();
    StatePose vehicle_pose0(7, p, q);
    p << 3.2, 2.2, 1.2;
    StatePose vehicle_pose1(7, p, q);

    // Set a trinocular bench with a baseline of 0.1 m
    StatePose sensor_pose0(7), sensor_pose1(7), sensor_pose2(7);
    sensor_pose0.p() << 0, 0, 0;
    sensor_pose0.q().setIdentity();
    sensor_pose1.p() << 0.1, 0, 0; // like a stereo bench
    sensor_pose1.q().setIdentity();
    sensor_pose2.p() << 0, 0.1, 0; // like a trinocular bench
    sensor_pose2.q().setIdentity();
    Vector4s intrinsic(320, 240, 320, 320); // VGA camera with 90deg HFoV
    PinHole sensor0(sensor_pose0, intrinsic);
    PinHole sensor1(sensor_pose1, intrinsic);
    PinHole sensor2(sensor_pose2, intrinsic);

    // Organize all sensors-, capture- and feature- pointers in map structures. Associations are automated using fake keys as follows.
    // The key is computed as follows (fr is frame index, cp is capture index, ft is feature index):
    // - for frames,   key = 1000*fr
    // - for captures, key = 1000*fr + 100*cp
    // - for features, key = 1000*fr + 100*cp + 10*ft
    typedef map<int, shared_ptr<PinHole> > SensorsMap;
    typedef map<int, shared_ptr<FeatureBase> > FeaturesMap;
    SensorsMap  sensors_map;
    FeaturesMap features_map;

    // Set the three sensors in the map
    sensors_map.insert(SensorsMap::value_type(  0, make_shared<PinHole>(sensor0) ) );
    sensors_map.insert(SensorsMap::value_type(100, make_shared<PinHole>(sensor1) ) );
    sensors_map.insert(SensorsMap::value_type(200, make_shared<PinHole>(sensor2) ) );

    // Pixel measurements - to be computed dynamically later on
    Vector2s pix;


    // 3. Build a tree with:
    int num_frames = 1; // frames,
    int num_captures = 3; // captures per frame,
    int num_features = 20; // features per capture,
    // num_correspondences = dynamic Correspondences to Features,
    // num_trans_sensors = dynamic TransSensors to Captures.

    // Vehicle is the root node
    auto vehicle_ptr = make_shared<VehicleBase>();

    // lower nodes: frames
    for (int frame_counter = 0; frame_counter < num_frames; frame_counter++)
    {
        vehicle_ptr->addFrame( make_shared<Frame>( vehicle_ptr, make_shared<StatePose>(vehicle_pose0), TimeStamp(0) ) );
        auto frame_ptr = *(vehicle_ptr->frameList().rbegin()); // get the last inserted element!

        // lower nodes : captures
        for (int capture_counter = 0; capture_counter < num_captures; capture_counter++)
        {
            int sensor_key = 100 * capture_counter;
            frame_ptr->addCapture(make_shared<CaptureImage>(frame_ptr, sensors_map.find(sensor_key)->second));

            auto capture_rev_iter = frame_ptr->captureList().rbegin(); // get the last inserted element!
            auto capture_ptr = *capture_rev_iter;

            // lower nodes: features
            for (int feature_counter = 0; feature_counter < num_features; feature_counter++)
            {

                pix = Vector2s(50 * (capture_counter + 1) + 10 * (feature_counter + 1), 5 * (capture_counter + 1) + 100 * (feature_counter + 1));
                pix += 50 * Vector2s::Random(); // XXX Random runs are now repeatable - we need to see how to change the random seed.

                capture_ptr->addFeature(make_shared<ImagePoint>(capture_ptr, pix));
                auto feature_rev_iter = capture_ptr->featureList().rbegin();
                auto feature_ptr = *feature_rev_iter;

                // This is only a fake to make feature association automatic here. Do not use it as example for real projects.
                int feature_key = 100 * capture_counter + 10 * feature_counter;
                features_map.insert(FeaturesMap::value_type(feature_key, feature_ptr));

                // lower nodes: correspondences
                // Guidelines:
                // 1 - link to all previous captures in the current frame
                // 2 - link to only the own capture in all previous frames
                // 3 - link always to the own feature !
                // 4 - try to add a TransSensor for each new link
                for (int corresp_counter = 0; corresp_counter < capture_counter; corresp_counter++) // implements 1, 3 and 4.
                {
                    // get a valid feature pointer which points to a different feature
                    int feature_other_key = (capture_counter - 1 - corresp_counter) * 100 + feature_counter * 10; // only to features of previous sensors
                    shared_ptr<FeatureBase> feature_other_ptr = features_map.find(feature_other_key)->second;

                    // add correspondence to the found feature
                    feature_ptr->addCorrespondence(make_shared<EpipolarConstraint>(feature_ptr, feature_other_ptr));
                    auto correspondence_ptr = *(feature_ptr->correspondenceList().rbegin());

                    // get correspondent Capture and its ID
                    auto capt_other_ptr = feature_other_ptr->capturePtr();
                    int capture_other_id = capt_other_ptr->nodeId();

                    // Add a TransSensor of type TransPinHole relating this and other Captures
                    if (!capture_ptr->existsTransSensor(capture_other_id)) // this is necessary to avoid unnecessary TransSensor construction.
                    {
                        capture_ptr->addTransSensor(capture_other_id,
                                                   make_shared<TransPinHole>(capture_ptr.get(), capt_other_ptr));
                    } // if ! existsTransSensor

                } // for c_k

            } // for f_j

        } // for C_i

    } // for F_i


    // 4. Simulate the start of one optimization

    // remap (to be done each time the state is modified)
    cout << "Vehicle's dim_error  = " << vehicle_ptr->computeDimError() << endl;
    error_storage.resize(vehicle_ptr->dimError());
    new (&error_storage_map) Map<VectorXs>(error_storage.data(), vehicle_ptr->dimError() );
    vehicle_ptr->remap(error_storage, 0);

    // simulate N iterations of the optimizer -- a large N is useful for profiling and get performance analysis
    unsigned int idx;
    for (unsigned int i = 0; i < N_iterations; i++)
    {
        // compute error (to be done once per optimizer iteration)
        vehicle_ptr->precomputeConstants(); // Only necessary if computeError() below does not take care of it.

        switch ( 5 )
        {
            case 1:
                // Use the mapped errors in the wolf nodes through all iterations
                vehicle_ptr->computeError();
                break;

            case 2:
                // Use the mapped errors in the wolf nodes but re-map at each iteration
                vehicle_ptr->remap(error_storage, 0);
                vehicle_ptr->computeError();
                break;

            case 3:
                // use an external error storage and a running index
                // no map
                idx = 0;
                vehicle_ptr->computeError(error_storage, idx);
                break;

            case 4:
                // use a Map to external error storage, and a running index
                idx = 0;
                vehicle_ptr->computeError(error_storage_map, idx);
                break;

            case 5:
                // use a pointer to error storage and a running index
                idx = 0;
                vehicle_ptr->computeError(error_storage.data(), idx);
                break;
        }
    }

    // 5. Print some results
    // recompute error once more with all methods to have all results regardless of the switch above
    //    vehicle_ptr->computeError();
    //    cout << "Vehicle error, node maps       = ( " << error_storage.transpose() << " )" << endl;
    //
    //    idx = 0;
    //    vehicle_ptr->computeError(error_storage, idx);
    //    cout << "Vehicle error, external vector = ( " << error_storage.transpose() << " )" << endl;
    //
    //    idx = 0;
    //    vehicle_ptr->computeError(error_storage_map, idx);
    //    cout << "Vehicle error, external map    = ( " << error_storage.transpose() << " )" << endl;

    idx = 0;
    vehicle_ptr->computeError(error_storage.data(), idx);
    cout << "Vehicle error, external pointer= ( " << error_storage.transpose() << " )" << endl;

    cout << endl;

    // print wolf tree ( edit option to (de)activate )
    if ( 1 )
    {
        // Use clean error vectors
        // We use an external pointer to compute errors, and then re-map the wolf tree over this external pointer.
        VectorXs error_storage_clean( vehicle_ptr->dimError() );
        idx = 0;
        vehicle_ptr->computeError(error_storage_clean.data(), idx); // compute with external pointer
        vehicle_ptr->remap(error_storage_clean.data(), 0); // re-map the node's error_s after the math is done
        vehicle_ptr->print();    cout << endl; // print to check that all errors are conveniently updated in the wolf tree.
    }

    return EXIT_SUCCESS;
}

