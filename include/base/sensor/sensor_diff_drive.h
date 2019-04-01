/**
 * \file sensor_diff_drive.h
 *
 *  Created on: Oct 20, 2016
 *  \author: Jeremie Deray
 */

#ifndef WOLF_SENSOR_DIFF_DRIVE_H_
#define WOLF_SENSOR_DIFF_DRIVE_H_

//wolf includes
#include "base/sensor/sensor_base.h"
#include "base/diff_drive_tools.h"
#include "base/params_server.hpp"

namespace wolf {

WOLF_STRUCT_PTR_TYPEDEFS(IntrinsicsDiffDrive);
WOLF_PTR_TYPEDEFS(SensorDiffDrive);

struct IntrinsicsDiffDrive : public IntrinsicsBase
{
  Scalar left_radius_;
  Scalar right_radius_;
  Scalar separation_;

  DiffDriveModel model_ = DiffDriveModel::Three_Factor_Model;

  Eigen::VectorXs factors_ = Eigen::Vector3s(1, 1, 1);

  Scalar left_resolution_;
  Scalar right_resolution_;

  Scalar left_gain_  = 0.01;
  Scalar right_gain_ = 0.01;

    IntrinsicsDiffDrive()
    {
        //DEFINED FOR COMPATIBILITY PURPOSES. TO BE REMOVED IN THE FUTURE.
    }
    IntrinsicsDiffDrive(std::string _unique_name, const paramsServer& _server):
        IntrinsicsBase(_unique_name, _server)
    {

        left_radius_ = _server.getParam<Scalar>(_unique_name + "/left_radius_");
        right_radius_ = _server.getParam<Scalar>(_unique_name + "/right_radius_");
        separation_ = _server.getParam<Scalar>(_unique_name + "/separation_");

        auto model_str = _server.getParam<std::string>(_unique_name + "/model");
        if(model_str.compare("Two_Factor_Model")) model_ = DiffDriveModel::Two_Factor_Model;
        else if(model_str.compare("Three_Factor_Model")) model_ = DiffDriveModel::Three_Factor_Model;
        else if(model_str.compare("Five_Factor_Model")) model_ = DiffDriveModel::Five_Factor_Model;
        else throw std::runtime_error("Failed to fetch a valid value for the enumerate DiffDriveModel. Value provided: " + model_str);

        factors_ = _server.getParam<Eigen::VectorXs>(_unique_name + "/factors", "[1,1,1]");

        left_resolution_ = _server.getParam<Scalar>(_unique_name + "/left_resolution_");
        right_resolution_ = _server.getParam<Scalar>(_unique_name + "/right_resolution_");

        left_gain_ = _server.getParam<Scalar>(_unique_name + "/left_gain", "0.01");
        right_gain_ = _server.getParam<Scalar>(_unique_name + "/right_gain", "0.01");
    }
  virtual ~IntrinsicsDiffDrive() = default;
};

typedef std::shared_ptr<IntrinsicsDiffDrive> IntrinsicsDiffDrivePtr;

class SensorDiffDrive : public SensorBase
{
public:

  /**
   * \brief Constructor with arguments
   *
   * Constructor with arguments
   * \param _p_ptr StateBlock pointer to the sensor position wrt vehicle base.
   * \param _o_ptr StateBlock pointer to the sensor orientation wrt vehicle base.
   * \param _i_ptr StateBlock pointer to the sensor dynamic intrinsics (factors).
   * \param _intrinsics The intrinsics parameters of the sensor.
   *
   **/
  SensorDiffDrive(const StateBlockPtr& _p_ptr,
                  const StateBlockPtr& _o_ptr,
                  const StateBlockPtr& _i_ptr,
                  const IntrinsicsDiffDrivePtr& _intrinsics);

  /**
   * \brief Default destructor (not recommended)
   **/
  virtual ~SensorDiffDrive() = default;

  inline IntrinsicsDiffDrivePtr getIntrinsics() const {return intrinsics_ptr_;}

protected:

  IntrinsicsDiffDrivePtr intrinsics_ptr_;

public:

  /**
   * @brief create. Factory function to create a SensorDiffDrive.
   * @param _unique_name. A unique name for the sensor.
   * @param _extrinsics_po. The (2d) position of the sensor w.r.t to the robot base frame.
   * @param _intrinsics. The sensor extrinsics parameters.
   * @return a SensorBasePtr holding the sensor. If the sensor creation failed,
   * the returned pointer is nullptr.
   */
  static SensorBasePtr create(const std::string& _unique_name,
                              const Eigen::VectorXs& _extrinsics_po,
                              const IntrinsicsBasePtr _intrinsics);
};

} // namespace wolf

#endif /* WOLF_SENSOR_DIFF_DRIVE_H_ */
