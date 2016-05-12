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
typedef Matrix<wolf::Scalar, 2, 2, Eigen::RowMajor> Matrix2s;                ///< 2x2 matrix of real Scalar type
typedef Matrix<wolf::Scalar, 3, 3, Eigen::RowMajor> Matrix3s;                ///< 3x3 matrix of real Scalar type
typedef Matrix<wolf::Scalar, 4, 4, Eigen::RowMajor> Matrix4s;                ///< 4x4 matrix of real Scalar type
typedef Matrix<wolf::Scalar, 7, 7, Eigen::RowMajor> Matrix7s;                ///< 7x7 matrix of real Scalar type
typedef Matrix<wolf::Scalar, Dynamic, Dynamic, Eigen::RowMajor> MatrixXs;    ///< variable size matrix of real Scalar type
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

/** \brief Enumeration of node locations at Wolf Tree
 *
 * You may add items to this list as needed. Be concise with names, and document your entries.
 */
typedef enum
{
    TOP = 1,   ///< root node location. This is the one that commands jobs down the tree.
    MID,   ///< middle nodes. These delegate jobs to lower nodes.
    BOTTOM ///< lowest level nodes. These are the ones that do not delegate any longer and have to do the job.
} NodeLocation;

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
    FRM_PQVBB_3D    ///< 3D frame with pos, orient quat, velocity, acc bias (abx,aby,abz), and gyro bias (wbx,wby,wbz).
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
    CTR_CORNER_2D,              ///< 2D corner constraint .
    CTR_CONTAINER,              ///< 2D container constraint .
    CTR_IMG_PNT_TO_EP,          ///< constraint from a image point to a Euclidean 3D point landmark (EP). See https://hal.archives-ouvertes.fr/hal-00451778/document
    CTR_IMG_PNT_TO_HP,          ///< constraint from a image point to a Homogeneous 3D point landmark (HP). See https://hal.archives-ouvertes.fr/hal-00451778/document
    CTR_EPIPOLAR                ///< Epipolar constraint
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
    PRC_TRACKER_DUMMY = 1,
    PRC_TRACKER_BRISK,
    PRC_TRACKER_ORB,
    PRC_TRACKER_LANDMARK_CORNER,
    PRC_TRACKER_FEATURE_CORNER,
    PRC_GPS_RAW,
    PRC_LIDAR,
    PRC_ODOM_2D,
    PRC_ODOM_3D,
    PRC_IMU
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
    FEATURE_ODOM_2D,
    FEATURE_MOTION,
    FEATURE_POINT_IMAGE, 
    FEATURE_LINE_2D
}FeatureType;

/** \brief Enumeration of all possible landmark types
 *
 */
typedef enum
{
    LANDMARK_POINT = 1,     ///< A Euclidean point landmark, either 3D or 2D
    LANDMARK_CORNER,    ///< A corner landmark (2D)
    LANDMARK_CONTAINER,  ///< A container landmark (2D)
    LANDMARK_LINE_2D
} LandmarkType;

typedef enum
{
    LANDMARK_CANDIDATE = 1,   ///< A landmark, just created. Association with it allowed, but not yet establish an actual constraint for the solver
    LANDMARK_ESTIMATED,   ///< A landmark being estimated. Association with it allowed, establishing actual constraints for the solver where both vehicle and landmark states are being estimated
    LANDMARK_FIXED,       ///< A landmark estimated. Association with it allowed, establishing actual constraints for the solver, but its value remains static, no longer optimized
    LANDMARK_OUT_OF_VIEW, ///< A landmark out of the field of view. Association with it is not allowed, so does not pose constraints for the solver
    LANDMARK_OLD          ///< An old landmark. Association with it not allowed, but old constraints can still be taken into account by the solver.
} LandmarkStatus;


/////////////////////////////////////////////////////////////////////////
//      TYPEDEFS FOR POINTERS AND ITERATORS IN THE WOLF TREE
/////////////////////////////////////////////////////////////////////////
// - forwards for pointers

class NodeTerminus;
class Problem;
class MapBase;
class LandmarkBase;
class LandmarkCorner2D;
class TrajectoryBase;
class FrameBase;
class CaptureBase;
class CaptureMotion;
class CaptureLaser2D;
class FeatureBase;
class FeatureCorner2D;
class ConstraintBase;
class SensorBase;
class SensorLaser2D;
class TransSensor;
class ProcessorBase;
class StateBlock;


// TODO: No seria millor que cada classe es defineixi aquests typedefs?

//Problem
typedef Problem* ProblemPtr;

//Map
typedef std::list<MapBase*> MapBaseList;
typedef MapBaseList::iterator MapBaseIter;

//Landmark
typedef std::list<LandmarkBase*> LandmarkBaseList;
typedef LandmarkBaseList::iterator LandmarkBaseIter;

//Landmark corner 2D
typedef std::list<LandmarkCorner2D*> LandmarkCorner2DList;
typedef LandmarkCorner2DList::iterator LandmarkCorner2DIter;

// - Frame
typedef std::list<FrameBase*> FrameBaseList;
typedef FrameBaseList::iterator FrameBaseIter;

// - Capture
typedef std::list<CaptureBase*> CaptureBaseList;
typedef CaptureBaseList::iterator CaptureBaseIter;

// - Capture Relative
typedef std::list<CaptureMotion*> CaptureRelativeList;
typedef CaptureRelativeList::iterator CaptureRelativeIter;

// - Feature
typedef std::list<FeatureBase*> FeatureBaseList;
typedef FeatureBaseList::iterator FeatureBaseIter;

// - Feature Corner 2D
typedef std::list<FeatureCorner2D*> FeatureCorner2DList;
typedef FeatureCorner2DList::iterator FeatureCorner2DIter;

// - Constraint
typedef std::list<ConstraintBase*> ConstraintBaseList;
typedef ConstraintBaseList::iterator ConstraintBaseIter;

// - Sensors
typedef std::list<SensorBase*> SensorBaseList;
typedef SensorBaseList::iterator SensorBaseIter;

// - transSensor
typedef std::map<unsigned int, TransSensor*> TransSensorMap;
typedef TransSensorMap::iterator TransSensorIter;

// - Processors
typedef std::list<ProcessorBase*> ProcessorBaseList;
typedef ProcessorBaseList::iterator ProcessorBaseIter;

// - State
typedef std::list<StateBlock*> StateBlockList;
typedef StateBlockList::iterator StateBlockIter;


// Match Feature - Landmark
struct LandmarkMatch
{
        LandmarkBase* landmark_ptr_;
        Scalar normalized_score_;
};

// Match map Feature - Landmark
typedef std::map<FeatureBase*, LandmarkMatch> LandmarkMatchMap;


inline Scalar pi2pi(const Scalar& angle)
{
    return (angle > 0 ? fmod(angle + M_PI, 2 * M_PI) - M_PI : fmod(angle - M_PI, 2 * M_PI) + M_PI);
}


// Feature-Feature correspondence
struct FeatureMatch
{
        FeatureBase* feature_ptr_;
        Scalar normalized_score_;
};

typedef std::map<FeatureBase*, FeatureMatch> FeatureMatchMap;

} // namespace wolf

// Quaternion things
namespace Eigen{
inline void v2q(const Eigen::VectorXs& _v, Eigen::Quaternions& _q){
    wolf::Scalar angle = _v.norm();
    if (angle < wolf::Constants::EPS)
        _q = Eigen::Quaternions::Identity();
    else
    {
        _q = Eigen::Quaternions(Eigen::AngleAxiss(angle, _v/angle));
    }
}

inline void v2q(const Eigen::VectorXs& _v, Eigen::Map<Eigen::Quaternions>& _q){
    wolf::Scalar angle = _v.norm();
    if (angle < wolf::Constants::EPS)
        _q = Eigen::Quaternions::Identity();
    else
    {
        _q = Eigen::Quaternions(Eigen::AngleAxiss(angle, _v/angle));
    }
}

inline void q2v(const Eigen::Quaternions& _q, Eigen::VectorXs& _v){
    Eigen::AngleAxiss aa = Eigen::AngleAxiss(_q);
    _v = aa.axis() * aa.angle();
}

inline void q2v(const Eigen::Map<const Eigen::Quaternions>& _q, Eigen::VectorXs& _v){
    Eigen::AngleAxiss aa = Eigen::AngleAxiss(_q);
    _v = aa.axis() * aa.angle();
}

}



#endif /* WOLF_H_ */
