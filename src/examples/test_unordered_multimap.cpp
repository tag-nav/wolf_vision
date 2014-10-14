/*
 * test_unordered_multimap.cpp
 *
 *  Created on: 15/06/2014
 *      \author: jsola
 */


#include <iostream>
#include <unordered_map>

#include "../vehicle_base.h"
#include "../sensor_imu.h"
#include "../sensor_pin_hole.h"
#include "../state_pq.h"
#include "../state_pqv.h"
#include "../state_imu.h"

using namespace std;


int main()
{
    cout << "\nNon-homogeneous sets: unordered_multimap demo." << endl;
    cout << "==============================================\n" << endl;
    cout << "1. Easy case from tutorial..., with char key and int value" << endl;
    typedef unordered_multimap<char,int> MyMap;
    MyMap map;
    map.insert(MyMap::value_type('a', 1));
    map.insert(MyMap::value_type('a', 1));
    map.insert(MyMap::value_type('b', 2));
    map.insert(MyMap::value_type('c', 3));
    map.insert(MyMap::value_type('d', 4));
    map.insert(MyMap::value_type('a', 7));
    map.insert(MyMap::value_type('b', 18));

    cout << "   all values to any key" << endl;
    for(auto it = map.begin(); it != map.end(); it++) {
        cout << it->first << '\t';
        cout << it->second << endl;
    }

    cout << "   all values to key a" << endl;
    auto its = map.equal_range('a');
    for (auto it = its.first; it != its.second; ++it) {
        cout << it->first << '\t' << it->second << endl;
    }

    cout << "CONCLUSION: easy to get elements by type." << endl;


    cout << "\n2. Now with char key and SensorBase value..." << endl;

    Vector3s p1(9,8,7), p2(2*p1), p3(3*p1), p4(4*p1);
    Quaternions q(1,2,3,4);
    q.normalize();
    StatePose pose1(p1,q), pose2(p2,q), pose3(p3,q), pose4(p4,q);
    Vector4s k1(1,2,3,4), k2(2*k1), k3(3*k1);
    VectorXs ki(6);
    ki << 6,5,4,3,2,1;
    SensorIMU imu1(pose1, ki), imu2(pose2);
    SensorPinHole camera1(pose3, k1), camera2(pose4, k2);

    typedef unordered_multimap<char, SensorBase> sensorSet;
    sensorSet sensors;

    sensors.insert(sensorSet::value_type('i', imu1));
    sensors.insert(sensorSet::value_type('i', imu2));
    sensors.insert(sensorSet::value_type('c', camera1));
    sensors.insert(sensorSet::value_type('c', camera2));

    cout << "   sensors set size: " << sensors.size() << endl;

    cout << "   intrinsics of sensors" << endl;
    for(auto it = sensors.begin(); it != sensors.end(); it++) {
        cout << it->first << '\t';
        cout << it->second.getIntrinsic().transpose() << endl;
    }

    cout << "   intrinsics of cameras" << endl;
    auto its2 = sensors.equal_range('c');
    for (auto it2 = its2.first; it2 != its2.second; it2++) {
        cout << it2->first << '\t' << it2->second.getIntrinsic().transpose() << endl;
    }


    unsigned int erased_cameras = sensors.erase(sensorSet::key_type('c'));
    cout << "   erased "<< erased_cameras << " cameras" << endl;
    cout << "   intrinsics of sensors" << endl;
    for(auto it = sensors.begin(); it != sensors.end(); it++) {
        cout << it->first << '\t';
        cout << it->second.getIntrinsic().transpose() << endl;
    }

    cout << "CONCLUSION: Allows different derived objects, one derived per key, without the use of pointers." << endl;
    cout << "CONCLUSION: Easy to erase elements by type. Need to iterate element-by-element to erase single elements." << endl;

    cout << "\n3. Trying with int key and automatically casting the enums to int..." << endl;
    cout << "   Sensor type enum: IMU = " << IMU << " ; CAMERA = " << CAMERA << endl;
    typedef unordered_multimap<int, SensorBase> sensorSetInt;
    sensorSetInt sensors2;

    sensors2.insert(sensorSet::value_type(IMU, imu1));
    sensors2.insert(sensorSet::value_type(IMU, imu2));
    sensors2.insert(sensorSet::value_type(CAMERA, camera1));
    sensors2.insert(sensorSet::value_type(CAMERA, camera2));

    cout << "   sensors set size: " << sensors.size() << endl;

    cout << "   intrinsics of sensors" << endl;
    for(auto it = sensors2.begin(); it != sensors2.end(); it++) {
        cout << it->first << '\t';
        cout << it->second.getIntrinsic().transpose() << endl;
    }

    cout << "   intrinsics of cameras" << endl;
    auto its3 = sensors2.equal_range(CAMERA);
    for (auto it2 = its3.first; it2 != its3.second; it2++) {
        cout << it2->first << '\t' << it2->second.getIntrinsic().transpose() << endl;
    }

    cout << "CONCLUSION: Allows built-in hasher for keys of type int." << endl;

    cout << "\n4. Now with vehicles and trajectory!" << endl;
    typedef unordered_multimap < SensorType , SensorBase, std::hash<int> > FeatMgrSet;
    VehicleBase<StatePQV, StateIMU> vehicle(2);
    vehicle.sensors_.insert(FeatMgrSet::value_type(CAMERA,camera1));
    vehicle.sensors_.insert(FeatMgrSet::value_type(CAMERA,camera2));
    vehicle.sensors_.insert(FeatMgrSet::value_type(IMU,imu1));
    vehicle.sensors_.insert(FeatMgrSet::value_type(IMU,imu2));

    cout << "   intrinsics of sensors" << endl;
    for(auto it = vehicle.sensors_.begin(); it != vehicle.sensors_.end(); it++) {
        cout << it->first << '\t';
        cout << it->second.getIntrinsic().transpose() << endl;
    }

    //    cout << "all values to key CAMERA" << endl;
    //    its = vehicle.sensors_.equal_range(CAMERA);
    //    for (auto it = its.first; it != its.second; ++it) {
    //        cout << it->first << '\t' << it->second << endl;
    //    }

    cout << "CONCLUSION: Need to implement the hasher for key of type enum. The same happens with string." << endl;

    cout << endl;
}

