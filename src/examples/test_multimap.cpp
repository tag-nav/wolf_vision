/*
 * test_multimap.cpp
 *
 *  Created on: Jun 16, 2014
 *      \author: jsola
 */





#include <iostream>
#include <map>

#include "../vehicle_base.h"
#include "../sensor_imu.h"
#include "../sensor_pin_hole.h"
#include "../state_pq.h"
#include "../state_pqv.h"
#include "../state_imu.h"

using namespace std;


int main()
{
    cout << "\nNon-homogeneous sets: multimap demo." << endl;
    cout << "====================================\n" << endl;
    cout << "1. Easy case from tutorial..., with char key and int value" << endl;
    typedef multimap<char, int> MyMap;
    MyMap map;
    map.insert(MyMap::value_type('a', 1));
    map.insert(MyMap::value_type('a', 1));
    map.insert(MyMap::value_type('b', 18));
    map.insert(MyMap::value_type('c', 3));
    map.insert(MyMap::value_type('d', 4));
    map.insert(MyMap::value_type('a', 7));
    map.insert(MyMap::value_type('b', 2));

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

    cout << "CONCLUSION: easy to get elements by type. " << endl;
    cout << "CONCLUSION: Keys are ordered (only keys, not values for each key)." << endl;


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

    typedef multimap<char, SensorBase> sensorSetChar;
    sensorSetChar sensors2;

    sensors2.insert(sensorSetChar::value_type('i', imu1));
    sensors2.insert(sensorSetChar::value_type('i', imu2));
    sensors2.insert(sensorSetChar::value_type('c', camera1));
    sensors2.insert(sensorSetChar::value_type('c', camera2));

    cout << "   sensors2 set size: " << sensors2.size() << endl;

    cout << "   intrinsics of sensors2" << endl;
    for(auto it = sensors2.begin(); it != sensors2.end(); it++) {
        cout << it->first << '\t';
        cout << it->second.getIntrinsic().transpose() << endl;
    }

    cout << "   intrinsics of cameras" << endl;
    auto its2 = sensors2.equal_range('c');
    for (auto it2 = its2.first; it2 != its2.second; it2++) {
        cout << it2->first << '\t' << it2->second.getIntrinsic().transpose() << endl;
    }


    unsigned int erased_cameras = sensors2.erase(sensorSetChar::key_type('c'));
    cout << "   erased "<< erased_cameras << " cameras" << endl;
    cout << "   intrinsics of sensors2" << endl;
    for(auto it = sensors2.begin(); it != sensors2.end(); it++) {
        cout << it->first << '\t';
        cout << it->second.getIntrinsic().transpose() << endl;
    }

    cout << "CONCLUSION: Allows different derived objects, one derived per key, without the use of pointers." << endl;
    cout << "CONCLUSION: Easy to erase elements by type. Need to iterate element-by-element to erase single elements." << endl;

    cout << "\n3. Trying with int key and automatically casting the enums to int..." << endl;
    cout << "   Sensor type enum: IMU = " << IMU << " ; CAMERA = " << CAMERA << endl;
    typedef multimap<int, SensorBase> sensorSetInt;
    sensorSetInt sensors3;

    sensors3.insert(sensorSetInt::value_type(IMU, imu1));
    sensors3.insert(sensorSetInt::value_type(IMU, imu2));
    sensors3.insert(sensorSetInt::value_type(CAMERA, camera1));
    sensors3.insert(sensorSetInt::value_type(CAMERA, camera2));

    cout << "   sensors set size: " << sensors3.size() << endl;

    cout << "   intrinsics of sensors" << endl;
    for(auto it = sensors3.begin(); it != sensors3.end(); it++) {
        cout << it->first << '\t';
        cout << it->second.getIntrinsic().transpose() << endl;
    }

    cout << "   intrinsics of cameras" << endl;
    auto its3 = sensors3.equal_range(CAMERA);
    for (auto it2 = its3.first; it2 != its3.second; it2++) {
        cout << it2->first << '\t' << it2->second.getIntrinsic().transpose() << endl;
    }

    cout << "CONCLUSION: Enum keys can be treated as int. See 4." << endl;

    cout << "\n4. Now with enum key and SensorBase!" << endl;
    typedef multimap<SensorType, SensorBase> sensorSetEnum;
    sensorSetEnum sensors4;
    sensors4.insert(sensorSetEnum::value_type(CAMERA,camera1));
    sensors4.insert(sensorSetEnum::value_type(CAMERA,camera2));
    sensors4.insert(sensorSetEnum::value_type(IMU,imu1));
    sensors4.insert(sensorSetEnum::value_type(IMU,imu2));

    cout << "   intrinsics of sensors" << endl;
    for(auto it = sensors4.begin(); it != sensors4.end(); it++) {
        cout << it->first << '\t';
        cout << it->second.getIntrinsic().transpose() << endl;
    }

    cout << "all values to key CAMERA" << endl;
    auto its4 = sensors4.equal_range(CAMERA);
    for (auto it = its4.first; it != its4.second; ++it) {
        cout << it->first << '\t' << it->second.getIntrinsic().transpose() << endl;
    }

    cout << "CONCLUSION: No need to implement any hasher for key of type enum." << endl;
    cout << "CONCLUSION: No need to cast enum to int as in 4. above." << endl;

    cout << "\n5. Now with string key and SensorBase!" << endl;
    typedef multimap<string, SensorBase> sensorSetString;
    sensorSetString sensors5;
    sensors5.insert(sensorSetString::value_type("CAMERA",camera1));
    sensors5.insert(sensorSetString::value_type("CAMERA",camera2));
    sensors5.insert(sensorSetString::value_type("0_IMU",imu1));
    sensors5.insert(sensorSetString::value_type("0_IMU",imu2));

    cout << "   intrinsics of sensors" << endl;
    for(auto it = sensors5.begin(); it != sensors5.end(); it++) {
        cout << it->first << '\t';
        cout << it->second.getIntrinsic().transpose() << endl;
    }

    cout << "all values to key CAMERA" << endl;
    auto its5 = sensors5.equal_range("CAMERA");
    for (auto it = its5.first; it != its5.second; ++it) {
        cout << it->first << '\t' << it->second.getIntrinsic().transpose() << endl;
    }

    cout << "CONCLUSION: Short strings may be a good way to categorize multimaps." << endl;
    cout << "CONCLUSION: If we want IMU to come first we need to force its string key as shown." << endl;

    cout << endl;
}

