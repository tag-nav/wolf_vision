/**
 * \file sensor_diff_drive.h
 *
 *  Created on: Oct 20, 2016
 *  \author: Jeremie Deray
 */

#ifndef WOLF_SENSOR_DIFF_DRIVE_H_
#define WOLF_SENSOR_DIFF_DRIVE_H_

//wolf includes
#include "../sensor_base.h"
#include "diff_drive_tools.h"

namespace wolf {

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
