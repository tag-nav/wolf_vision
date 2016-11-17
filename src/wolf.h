/**
 * \file wolf.h
 * \brief General typedefs for the Wolf project
 * \author Joan Sola
 *  Created on: 28/05/2014
 */

#ifndef WOLF_H_
#define WOLF_H_

// Enable project-specific definitions and macros
#include "internal/config.h"
#include "logging.h"

//includes from Eigen lib
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Sparse>
#include <libgen.h>

//includes from std lib
#include <list>
#include <map>
#include <memory> // shared_ptr and weak_ptr

namespace wolf {

/**
 * \brief scalar type for the Wolf project
 *
 * This typedef makes it possible to recompile the whole Wolf project with double, float, or other precisions.
 *
 * To change the project precision, just edit wolf.h and change the precision of this typedef.
 *
 * Do NEVER forget to use Wolf scalar definitions when you code!!!
 *
 * Do NEVER use plain Eigen scalar types!!! (This is redundant with the above. But just to make sure you behave!)
 *
 * The ONLY exception to this rule is when you need special precision. The ONLY example by now is the time stamp which uses double.
 */
//typedef float Scalar;         // Use this for float, 32 bit precision
typedef double Scalar;        // Use this for double, 64 bit precision
//typedef long double Scalar;   // Use this for long double, 128 bit precision


/**
 * \brief Vector and Matrices size type for the Wolf project
 *
 * We use the default defined in Eigen (int)
 *
 */
typedef EIGEN_DEFAULT_DENSE_INDEX_TYPE Size;

namespace Constants{

// Wolf standard tolerance
const Scalar EPS = 1e-8;
// Wolf smmmmall tolerance
const Scalar EPS_SMALL = 1e-16;

}

} // namespace wolf

///////////////////////////////////////////
// Construct types for any scalar defined in the typedef Scalar above
////////////////////////////////////////////
/** \brief Namespace extending Eigen definitions
 *
 * We redefine all algebra and gemoetry types to hold double or single precision floats.
 * The appended letter indicating this is 's', so that we have, e.g.,
 * - VectorXf   Vector of floats - defined by Eigen
 * - VectorXd   Vector of doubles - defined by Eigen
 * - VectorXs   Vector of either double of float, depending on the type \b Scalar, defined by Wolf.
 *
 */
namespace Eigen  // Eigen namespace extension
{
// 1. Vectors and Matrices
typedef Matrix<wolf::Scalar, 2, 2, RowMajor> Matrix2s;                ///< 2x2 matrix of real Scalar type
typedef Matrix<wolf::Scalar, 3, 3, RowMajor> Matrix3s;                ///< 3x3 matrix of real Scalar type
typedef Matrix<wolf::Scalar, 4, 4, RowMajor> Matrix4s;                ///< 4x4 matrix of real Scalar type
typedef Matrix<wolf::Scalar, 6, 6, RowMajor> Matrix6s;                ///< 6x6 matrix of real Scalar type
typedef Matrix<wolf::Scalar, 7, 7, RowMajor> Matrix7s;                ///< 7x7 matrix of real Scalar type
typedef Matrix<wolf::Scalar, Dynamic, Dynamic, RowMajor> MatrixXs;    ///< variable size matrix of real Scalar type
typedef Matrix<wolf::Scalar, 1, 1> Vector1s;                ///< 1-vector of real Scalar type
typedef Matrix<wolf::Scalar, 2, 1> Vector2s;                ///< 2-vector of real Scalar type
typedef Matrix<wolf::Scalar, 3, 1> Vector3s;                ///< 3-vector of real Scalar type
typedef Matrix<wolf::Scalar, 4, 1> Vector4s;                ///< 4-vector of real Scalar type
typedef Matrix<wolf::Scalar, 6, 1> Vector6s;                ///< 6-vector of real Scalar type
typedef Matrix<wolf::Scalar, 7, 1> Vector7s;                ///< 7-vector of real Scalar type
typedef Matrix<wolf::Scalar, Dynamic, 1> VectorXs;          ///< variable size vector of real Scalar type
typedef Matrix<wolf::Scalar, 1, 2> RowVector2s;             ///< 2-row-vector of real Scalar type
typedef Matrix<wolf::Scalar, 1, 3> RowVector3s;             ///< 3-row-vector of real Scalar type
typedef Matrix<wolf::Scalar, 1, 4> RowVector4s;             ///< 4-row-vector of real Scalar type
typedef Matrix<wolf::Scalar, 1, 7> RowVector7s;             ///< 7-row-vector of real Scalar type
typedef Matrix<wolf::Scalar, 1, Dynamic> RowVectorXs;       ///< variable size row-vector of real Scalar type

// 2. Quaternions and other rotation things
typedef Quaternion<wolf::Scalar> Quaternions;               ///< Quaternion of real Scalar type
typedef AngleAxis<wolf::Scalar> AngleAxiss;                 ///< Angle-Axis of real Scalar type
typedef Rotation2D<wolf::Scalar> Rotation2Ds;               ///< Rotation2D of real Scalar type
}

namespace wolf {

/** \brief Enumeration of frame types: key-frame or non-key-frame
 */
typedef enum
{
    NON_KEY_FRAME = 0,  ///< regular frame. It does play at optimizations but it will be discarded from the window once a newer frame arrives.
    KEY_FRAME = 1       ///< key frame. It will stay in the frames window and play at optimizations.
} FrameType;

/** \brief Enumeration of all possible frames
 *
 * You may add items to this list as needed. Be concise with names, and document your entries.
 */
typedef enum
{
    FRM_PO_2D = 1,  ///< 2D frame containing position (x,y) and orientation angle.
    FRM_PO_3D,      ///< 3D frame containing position (x,y,z) and orientation quaternion (qx,qy,qz,qw).
    FRM_POV_3D,     ///< 3D frame with position, orientation quaternion, and linear velocity (vx,vy,vz)
    FRM_PVQBB_3D    ///< 3D frame with pos, velocity, orient quat, acc bias (abx,aby,abz), and gyro bias (wbx,wby,wbz).
} FrameStructure;

/** \brief Enumeration of all possible constraints
 *
 * You may add items to this list as needed. Be concise with names, and document your entries.
 */
typedef enum
{
    CTR_GPS_FIX_2D = 1,         ///< 2D GPS Fix constraint.
    CTR_GPS_PR_2D,              ///< 2D GPS Pseudorange constraint.
    CTR_GPS_PR_3D,              ///< 3D GPS Pseudorange constraint.
    CTR_FIX,                    ///< Fix constraint (for priors).
    CTR_FIX_3D,                    ///< Fix constraint (for priors) in 3D.
    CTR_ODOM_2D,                ///< 2D Odometry constraint .
    CTR_ODOM_3D,                ///< 3D Odometry constraint .
    CTR_CORNER_2D,              ///< 2D corner constraint .
    CTR_POINT_2D,               ///< 2D point constraint .
    CTR_POINT_TO_LINE_2D,       ///< 2D point constraint .
    CTR_CONTAINER,              ///< 2D container constraint .
    CTR_IMG_PNT_TO_EP,          ///< constraint from a image point to a Euclidean 3D point landmark (EP). See https://hal.archives-ouvertes.fr/hal-00451778/document
    CTR_IMG_PNT_TO_HP,          ///< constraint from a image point to a Homogeneous 3D point landmark (HP). See https://hal.archives-ouvertes.fr/hal-00451778/document
    CTR_EPIPOLAR,               ///< Epipolar constraint
    CTR_AHP,                    ///< Anchored Homogeneous Point constraint
    CTR_AHP_NL,                 ///< Anchored Homogeneous Point constraint (temporal, to be removed)
    CTR_IMU                     ///< IMU constraint

} ConstraintType;

/** \brief Enumeration of constraint categories
 *
 * You may add items to this list as needed. Be concise with names, and document your entries.
 */
typedef enum
{
    CTR_ABSOLUTE = 1,   ///< Constraint established with absolute reference.
    CTR_FRAME,          ///< Constraint established with a frame (odometry).
    CTR_FEATURE,        ///< Constraint established with a feature (correspondence).
    CTR_LANDMARK        ///< Constraint established with a landmark (correpondence).
} ConstraintCategory;

/** \brief Enumeration of constraint status
 *
 * You may add items to this list as needed. Be concise with names, and document your entries.
 */
typedef enum
{
    CTR_INACTIVE = 0,   ///< Constraint established with a frame (odometry).
    CTR_ACTIVE = 1      ///< Constraint established with absolute reference.
} ConstraintStatus;

/** \brief Enumeration of jacobian computation method
 *
 * You may add items to this list as needed. Be concise with names, and document your entries.
 */
typedef enum
{
    JAC_AUTO = 1,   ///< Auto differentiation (AutoDiffCostFunctionWrapper or ceres::NumericDiffCostFunction).
    JAC_NUMERIC,    ///< Numeric differentiation (ceres::NumericDiffCostFunction).
    JAC_ANALYTIC    ///< Analytic jacobians.
} JacobianMethod;


/** \brief Enumeration of all possible state status
 *
 * You may add items to this list as needed. Be concise with names, and document your entries.
 */
typedef enum
{
    ST_ESTIMATED = 0,   ///< State in estimation (default)
    ST_FIXED = 1,       ///< State fixed, estimated enough or fixed infrastructure.
} StateStatus;


typedef enum
{
    LANDMARK_CANDIDATE = 1,   ///< A landmark, just created. Association with it allowed, but not yet establish an actual constraint for the solver
    LANDMARK_ESTIMATED,   ///< A landmark being estimated. Association with it allowed, establishing actual constraints for the solver where both vehicle and landmark states are being estimated
    LANDMARK_FIXED,       ///< A landmark estimated. Association with it allowed, establishing actual constraints for the solver, but its value remains static, no longer optimized
} LandmarkStatus;


/////////////////////////////////////////////////////////////////////////
//      TYPEDEFS FOR POINTERS, LISTS AND ITERATORS IN THE WOLF TREE
/////////////////////////////////////////////////////////////////////////

// - forwards for pointers
class NodeBase;
class Problem;
class HardwareBase;
class SensorBase;
struct IntrinsicsBase;
class ProcessorBase;
class ProcessorMotion;
struct ProcessorParamsBase;
class TrajectoryBase;
class FrameBase;
class CaptureBase;
class CaptureMotion;
class FeatureBase;
class ConstraintBase;
class MapBase;
class LandmarkBase;
class StateBlock;
class StateQuaternion;
class LocalParametrizationBase;

// NodeBase
typedef std::shared_ptr<NodeBase> NodeBasePtr;
typedef std::weak_ptr<NodeBase> NodeBaseWPtr;

// Problem
typedef std::shared_ptr<Problem> ProblemPtr;
typedef std::weak_ptr<Problem> ProblemWPtr;

// Hardware
typedef std::shared_ptr<HardwareBase> HardwareBasePtr;
typedef std::weak_ptr<HardwareBase> HardwareBaseWPtr;

// - Sensors
typedef std::shared_ptr<SensorBase> SensorBasePtr;
typedef std::weak_ptr<SensorBase> SensorBaseWPtr;
typedef std::list<SensorBasePtr> SensorBaseList;
typedef SensorBaseList::iterator SensorBaseIter;

// - - Intrinsics
typedef std::shared_ptr<IntrinsicsBase> IntrinsicsBasePtr;

// - Processors
typedef std::shared_ptr<ProcessorBase> ProcessorBasePtr;
typedef std::weak_ptr<ProcessorBase> ProcessorBaseWPtr;
typedef std::list<ProcessorBasePtr> ProcessorBaseList;
typedef ProcessorBaseList::iterator ProcessorBaseIter;

// - ProcessorMotion
typedef std::shared_ptr<ProcessorMotion> ProcessorMotionPtr;
typedef std::weak_ptr<ProcessorMotion> ProcessorMotionWPtr;

// - - Processor params
typedef std::shared_ptr<ProcessorParamsBase> ProcessorParamsBasePtr;

// Trajectory
typedef std::shared_ptr<TrajectoryBase> TrajectoryBasePtr;
typedef std::weak_ptr<TrajectoryBase> TrajectoryBaseWPtr;

// - Frame
typedef std::shared_ptr<FrameBase> FrameBasePtr;
typedef std::weak_ptr<FrameBase> FrameBaseWPtr;
typedef std::list<FrameBasePtr> FrameBaseList;
typedef FrameBaseList::iterator FrameBaseIter;

// - Capture
typedef std::shared_ptr<CaptureBase> CaptureBasePtr;
typedef std::weak_ptr<CaptureBase> CaptureBaseWPtr;
typedef std::list<CaptureBasePtr> CaptureBaseList;
typedef CaptureBaseList::iterator CaptureBaseIter;

// - Feature
typedef std::shared_ptr<FeatureBase> FeatureBasePtr;
typedef std::weak_ptr<FeatureBase> FeatureBaseWPtr;
typedef std::list<FeatureBasePtr> FeatureBaseList;
typedef FeatureBaseList::iterator FeatureBaseIter;

// - Constraint
typedef std::shared_ptr<ConstraintBase> ConstraintBasePtr;
typedef std::weak_ptr<ConstraintBase> ConstraintBaseWPtr;
typedef std::list<ConstraintBasePtr> ConstraintBaseList;
typedef ConstraintBaseList::iterator ConstraintBaseIter;

// Map
typedef std::shared_ptr<MapBase> MapBasePtr;
typedef std::weak_ptr<MapBase> MapBaseWPtr;
typedef std::list<MapBasePtr> MapBaseList;
typedef MapBaseList::iterator MapBaseIter;

// - Landmark
typedef std::shared_ptr<LandmarkBase> LandmarkBasePtr;
typedef std::weak_ptr<LandmarkBase> LandmarkBaseWPtr;
typedef std::list<LandmarkBasePtr> LandmarkBaseList;
typedef LandmarkBaseList::iterator LandmarkBaseIter;

// - - State blocks
typedef std::shared_ptr<StateBlock> StateBlockPtr;
typedef std::weak_ptr<StateBlock> StateBlockWPtr;
typedef std::list<StateBlockPtr> StateBlockList;
typedef StateBlockList::iterator StateBlockIter;
typedef std::shared_ptr<StateQuaternion> StateQuaternionPtr;

// - - Local Parametrization
typedef std::shared_ptr<LocalParametrizationBase> LocalParametrizationBasePtr;


// ==================================================
// Some dangling functions

inline const Eigen::Vector3s gravity(void) {
    return Eigen::Vector3s(0,0,-9.8);
}
//===================================================


//===================================================
// Some macros

#define WOLF_DEBUG_HERE \
{ \
    char this_file[] = __FILE__; \
    std::cout << ">> " << basename(this_file) << " : " << __FUNCTION__ << "() : " << __LINE__ << std::endl; \
}
//===================================================

} // namespace wolf



#endif /* WOLF_H_ */
