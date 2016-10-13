/**
 * \file wolf.h
 * \brief General typedefs for the Wolf project
 * \author Joan Sola
 *  Created on: 28/05/2014
 */

#ifndef WOLF_H_
#define WOLF_H_

//includes from std lib
#include <list>
#include <map>

//includes from Eigen lib
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Sparse>

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

// use it in odometry covariances for instance.
//const Scalar MIN_VARIANCE = 1e-6; // 9/5/16: Delete this after 9/6/16 if nobody complains

//const Scalar PI = 3.14159265358979323846264338328; // Use M_PI from math.h. There are a bunch of other useful constants there.

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

/** \brief Enumeration of all possible frames
 *
 * You may add items to this list as needed. Be concise with names, and document your entries.
 */
typedef enum
{
    NON_KEY_FRAME = 0,  ///< regular frame. It does play at optimizations but it will be discarded from the window once a newer frame arrives.
    KEY_FRAME = 1       ///< key frame. It will stay in the frames window and play at optimizations.
} FrameKeyType;

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

/** \brief Enumeration of all possible sensor types
 *
 * You may add items to this list as needed. Be concise with names, and document your entries.
 */
typedef enum
{
    SEN_ODOM_2D = 1,    ///< 2D Odometry measurement from encoders: displacement and rotation.
    SEN_ODOM_3D,        ///< 3D Odometry measurement from encoders: displacement and rotation.
    SEN_TWIST_2D,       ///< Twist measurement form encoders or motion command: lineal and angular velocities.
    SEN_IMU,		    ///< Inertial measurement unit with 3 acceleros, 3 gyros
    SEN_CAMERA,		    ///< Regular pinhole camera
    SEN_GPS_FIX,	    ///< GPS fix calculated from a GPS receiver
    SEN_GPS_RAW,        ///< GPS pseudo ranges, Doppler and satellite ephemerides
    SEN_LIDAR,		    ///< Laser Range Finder, 2D
    SEN_RADAR,		    ///< Radar
    SEN_ABSOLUTE_POSE   ///< Full absolute pose (XYZ+quaternion)
} SensorType;

/** \brief Enumeration of all possible Processor types
 *
 * You may add items to this list as needed. Be concise with names, and document your entries.
 */
typedef enum
{
    PRC_TRACKER_DUMMY = 1, ///< Dummy tracker for tests
    PRC_TRACKER_IMAGE, ///< Point feature tracker for video sequences
    PRC_TRACKER_LANDMARK_CORNER, ///< Tracker of corner Landmarks
    PRC_TRACKER_FEATURE_CORNER, ///<  Tracker of corner Features
    PRC_GPS_RAW, ///< Raw GPS processor
    PRC_LIDAR, ///< Laser 2D processor
    PRC_ODOM_2D, ///< 2D odometry integrator
    PRC_ODOM_3D, ///< 2D odometry integrator
    PRC_IMU ///< IMU delta pre-integrator
} ProcessorType;

/** \brief enumeration of all possible Feature types
 *
 * You may add items to this list as needed. Be concise with names, and document your entries.
 */
typedef enum
{
    FEATURE_CORNER = 1,
    FEATURE_FIX,
    FEATURE_GPS_FIX,
    FEATURE_GPS_PSEUDORANGE,
    FEATURE_IMU,
    FEATURE_ODOM_2D,
    FEATURE_MOTION,
    FEATURE_POINT_IMAGE,
    FEATURE_LINE_2D,
    FEATURE_POLYLINE_2D
}FeatureType;

/** \brief Enumeration of all possible landmark types
 *
 */
typedef enum
{
    LANDMARK_POINT = 1,     ///< A Euclidean point landmark, either 3D or 2D
    LANDMARK_CORNER,    ///< A corner landmark (2D)
    LANDMARK_CONTAINER,  ///< A container landmark (2D)
    LANDMARK_LINE_2D,  ///< A line landmark (2D)
    LANDMARK_POLYLINE_2D,   ///< A polyline landmark (2D)
    LANDMARK_AHP        ///< An anchored homogeneous point (3D)
} LandmarkType;

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
typedef NodeBase* NodeBasePtr;

//Problem
typedef Problem* ProblemPtr;

// Hardware
typedef HardwareBase* HardwareBasePtr;

// - Sensors
typedef SensorBase* SensorBasePtr;
typedef std::list<SensorBase*> SensorBaseList;
typedef SensorBaseList::iterator SensorBaseIter;

// Intrinsics
typedef IntrinsicsBase* IntrinsicsBasePtr;

// - Processors
typedef ProcessorBase* ProcessorBasePtr;
typedef std::list<ProcessorBase*> ProcessorBaseList;
typedef ProcessorBaseList::iterator ProcessorBaseIter;

// Processor params
typedef ProcessorParamsBase* ProcessorParamsBasePtr;

// - Trajectory
typedef TrajectoryBase* TrajectoryBasePtr;

// - Frame
typedef FrameBase* FrameBasePtr;
typedef std::list<FrameBase*> FrameBaseList;
typedef FrameBaseList::iterator FrameBaseIter;

// - Capture
typedef CaptureBase* CaptureBasePtr;
typedef std::list<CaptureBase*> CaptureBaseList;
typedef CaptureBaseList::iterator CaptureBaseIter;

// - Capture Relative
typedef std::list<CaptureMotion*> CaptureRelativeList;
typedef CaptureRelativeList::iterator CaptureRelativeIter;

// - Feature
typedef FeatureBase* FeatureBasePtr;
typedef std::list<FeatureBase*> FeatureBaseList;
typedef FeatureBaseList::iterator FeatureBaseIter;

// - Constraint
typedef ConstraintBase* ConstraintBasePtr;
typedef std::list<ConstraintBase*> ConstraintBaseList;
typedef ConstraintBaseList::iterator ConstraintBaseIter;

//Map
typedef MapBase* MapBasePtr;
typedef std::list<MapBasePtr> MapBaseList;
typedef MapBaseList::iterator MapBaseIter;

//Landmark
typedef LandmarkBase* LandmarkBasePtr;
typedef std::list<LandmarkBasePtr> LandmarkBaseList;
typedef LandmarkBaseList::iterator LandmarkBaseIter;

// - State blocks
typedef StateBlock* StateBlockPtr;
typedef std::list<StateBlockPtr> StateBlockList;
typedef StateBlockList::iterator StateBlockIter;
typedef StateQuaternion* StateQuaternionPtr;

// Local Parametrization
typedef LocalParametrizationBase* LocalParametrizationBasePtr;

// Match Feature - Landmark
struct LandmarkMatch
{
        LandmarkBasePtr landmark_ptr_;
        Scalar normalized_score_;
};

// Match map Feature - Landmark
typedef std::map<FeatureBasePtr, LandmarkMatch*> LandmarkMatchMap;


// Feature-Feature correspondence
struct FeatureMatch
{
        FeatureBasePtr feature_ptr_;
        Scalar normalized_score_;
};

typedef std::map<FeatureBasePtr, FeatureMatch> FeatureMatchMap;



inline const Eigen::Vector3s gravity(void) {
    return Eigen::Vector3s(0,0,-9.8);
}


} // namespace wolf



#endif /* WOLF_H_ */
