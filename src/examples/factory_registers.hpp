/**
 * \file factory_registers.h
 *
 *  Created on: May 6, 2016
 *      \author: jsola
 */

#ifndef FACTORY_REGISTERS_HPP_
#define FACTORY_REGISTERS_HPP_


namespace {
// Register factories at global scope!
const bool registered_sen_camera    = wolf::SensorFactory::get()->registerCreator("CAMERA",     wolf::SensorCamera::create);
const bool registered_sen_odom_2d   = wolf::SensorFactory::get()->registerCreator("ODOM 2D",    wolf::SensorOdom2D::create);
const bool registered_sen_gps_fix   = wolf::SensorFactory::get()->registerCreator("GPS FIX",    wolf::SensorGPSFix::create);
const bool registered_sen_gps       = wolf::SensorFactory::get()->registerCreator("GPS",        wolf::SensorGPS::create);
const bool registered_sen_imu       = wolf::SensorFactory::get()->registerCreator("IMU",        wolf::SensorIMU::create);
//const bool registered_laser         = wolf::SensorFactory::get()->registerCreator("LASER 2D",   wolf::SensorLaser2D::create);

const bool registered_prc_odom_3d   = wolf::ProcessorFactory::get()->registerCreator("ODOM 3D", wolf::ProcessorOdom3D::create);
const bool registered_prc_imu       = wolf::ProcessorFactory::get()->registerCreator("IMU",     wolf::ProcessorIMU::create);
const bool registered_prc_odom_2d   = wolf::ProcessorFactory::get()->registerCreator("ODOM 2D", wolf::ProcessorOdom2D::create);
//const bool registered_prc_laser_2d  = wolf::ProcessorFactory::get()->registerCreator("LASER 2D", wolf::ProcessorLaser2D::create);
//const bool registered_prc_gps       = wolf::ProcessorFactory::get()->registerCreator("GPS",     wolf::ProcessorGPS::create);
}



#endif /* FACTORY_REGISTERS_HPP_ */
