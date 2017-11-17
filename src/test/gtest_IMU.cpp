/*
 * gtest_IMU.cpp
 *
 *  Created on: Nov 14, 2017
 *      Author: jsola
 */

//Wolf
#include "wolf.h"
#include "sensor_imu.h"
#include "processor_imu.h"
#include "sensor_odom_3D.h"
#include "processor_odom_3D.h"
#include "ceres_wrapper/ceres_manager.h"

#include "utils_gtest.h"
#include "logging.h"

// make my life easier
using namespace Eigen;
using namespace wolf;
using std::shared_ptr;
using std::make_shared;
using std::static_pointer_cast;
using std::string;


class Process_Constraint_IMU : public testing::Test
{
    public:
        // Wolf objects
        ProblemPtr          problem;
        SensorIMUPtr        sensor_imu;
        ProcessorIMUPtr     processor_imu;
        FrameBasePtr        KF_0, KF_1;
        CaptureBasePtr      C_0, C_1;
        CaptureMotionPtr    CM_0, CM_1;
        CaptureIMUPtr       capture_imu;
        CeresManagerPtr     ceres_manager;

        // time
        TimeStamp           t0, t;
        Scalar              dt, DT;
        int                 num_integrations;

        // initial state
        VectorXs            x0;
        Vector3s            p0, v0;
        Quaternions         q0, q;
        Matrix<Scalar, 9, 9> P0;

        // bias
        Vector6s            bias_real, bias_preint, bias_null;
        Vector6s            bias_0, bias_1;                    // optimized bias at KF's 0 and 1

        // input
        Vector6s            motion, data;
        Vector3s            a, w;                              // true acc and angvel in IMU frame

        // Deltas and states (exact, integrated, corrected, solved, etc)
        VectorXs            D_exact, x1_exact;                 // exact or ground truth
        VectorXs            D_preint_imu, x1_preint_imu;       // preintegrated with imu_tools
        VectorXs            D_corrected_imu, x1_corrected_imu; // corrected with imu_tools
        VectorXs            D_preint, x1_preint;               // preintegrated with processor_imu
        VectorXs            D_corrected, x1_corrected;         // corrected with processor_imu
        VectorXs            D_optim, x1_optim;                 // optimized using constraint_imu
        VectorXs            D_optim_imu, x1_optim_imu;         // corrected with imu_tools using optimized bias
        VectorXs            x0_optim;                          // optimized using constraint_imu

        // Delta correction Jacobian and step
        Matrix<Scalar, 9, 6> J_D_bias;
        Vector9s            step;

        // Flags for fixing/unfixing state blocks
        bool                p0_fixed, q0_fixed, v0_fixed;
        bool                p1_fixed, q1_fixed, v1_fixed;



        virtual void SetUp( )
        {
            string wolf_root = _WOLF_ROOT_DIR;

            //===================================== SETTING PROBLEM
            problem = Problem::create("POV 3D");

            // CERES WRAPPER
            ceres::Solver::Options ceres_options;
            ceres_manager = make_shared<CeresManager>(problem, ceres_options);

            // SENSOR + PROCESSOR IMU
            SensorBasePtr       sensor = problem->installSensor   ("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
            ProcessorBasePtr processor = problem->installProcessor("IMU", "IMU pre-integrator", "Main IMU", wolf_root + "/src/examples/processor_imu_no_vote.yaml");
            sensor_imu    = static_pointer_cast<SensorIMU>   (sensor);
            processor_imu = static_pointer_cast<ProcessorIMU>(processor);

            // Some initializations
            bias_null.setZero();
            x0.resize(10);
            D_preint.resize(10);
            D_corrected.resize(10);

        }


        /* Create IMU data from body motion
         * Input:
         *   motion: [ax, ay, az, wx, wy, wz] the motion in body frame
         *   q: the current orientation wrt horizontal
         *   bias: the bias of the IMU
         * Output:
         *   return: the data vector as created by the IMU (with motion, gravity, and bias)
         */
        VectorXs motion2data(const VectorXs& body, const Quaternions& q, const VectorXs& bias)
        {
            VectorXs data(6);
            data          = body;                       // start with body motion
            data.head(3) -= q.conjugate()*gravity();    // add -g
            data         += bias;                       // add bias
            return data;
        }


        /* Integrate acc and angVel motion, obtain Delta_preintegrated
         * Input:
         *   N: number of steps
         *   q0: initial orientaiton
         *   motion: [ax, ay, az, wx, wy, wz] as the true magnitudes in body brame
         *   bias_real: the real bias of the IMU
         *   bias_preint: the bias used for Delta pre-integration
         * Output:
         *   return: the preintegrated delta
         */
        VectorXs integrateDelta(int N, const Quaternions& q0, const VectorXs& motion, const VectorXs& bias_real, const VectorXs& bias_preint, Scalar dt)
        {
            VectorXs data(6);
            VectorXs body(6);
            VectorXs delta(10);
            VectorXs Delta(10), Delta_plus(10);
            Delta = imu::identity();
            Quaternions q(q0);
            for (int n = 0; n<N; n++)
            {
                data        = motion2data(motion, q, bias_real);
                q           = q*exp_q(motion.tail(3)*dt);
                body        = data - bias_preint;
                delta       = imu::body2delta(body, dt);
                Delta_plus  = imu::compose(Delta, delta, dt);
                Delta       = Delta_plus;
            }
            return Delta;
        }

        /* Integrate acc and angVel motion, obtain Delta_preintegrated
         * Input:
         *   N: number of steps
         *   q0: initial orientaiton
         *   motion: [ax, ay, az, wx, wy, wz] as the true magnitudes in body brame
         *   bias_real: the real bias of the IMU
         *   bias_preint: the bias used for Delta pre-integration
         * Output:
         *   J_D_b: the Jacobian of the preintegrated delta wrt the bias
         *   return: the preintegrated delta
         */
        VectorXs integrateDelta(int N, const Quaternions& q0, const VectorXs& motion, const VectorXs& bias_real, const VectorXs& bias_preint, Scalar dt, Matrix<Scalar, 9, 6>& J_D_b)
        {
            VectorXs data(6);
            VectorXs body(6);
            VectorXs delta(10);
            Matrix<Scalar, 9, 6> J_d_d, J_d_b;
            Matrix<Scalar, 9, 9> J_D_D, J_D_d;
            VectorXs Delta(10), Delta_plus(10);
            Quaternions q;

            Delta = imu::identity();
            J_D_b.setZero();
            q = q0;
            for (int n = 0; n<N; n++)
            {
                // Simulate data
                data = motion2data(motion, q, bias_real);
                q    = q*exp_q(motion.tail(3)*dt);

                // Motion::integrateOneStep()
                {   // IMU::computeCurrentDelta
                    body  = data - bias_preint;
                    imu::body2delta(body, dt, delta, J_d_d);
                    J_d_b = - J_d_d;
                }
                {   // IMU::deltaPlusDelta
                    imu::compose(Delta, delta, dt, Delta_plus, J_D_D, J_D_d);
                }
                // Motion:: jac calib
                J_D_b = J_D_D*J_D_b + J_D_d*J_d_b;
                // Motion:: buffer
                Delta = Delta_plus;
            }
            return Delta;
        }

        void integrateWithProcessor(int N, const TimeStamp& t0, const Quaternions q0, const VectorXs& motion, const VectorXs& bias_real, const VectorXs& bias_preint, Scalar dt, VectorXs& D_preint, VectorXs& D_corrected)
        {
            data = motion2data(motion, q0, bias_real);
            capture_imu = make_shared<CaptureIMU>(t0, sensor_imu, data, sensor_imu->getNoiseCov());
            q = q0;
            t = t0;
            for (int i= 0; i < N; i++)
            {
                t   += dt;
                data = motion2data(motion, q, bias_real);
                q    = q * exp_q(w*dt);

                capture_imu->setTimeStamp(t);
                capture_imu->setData(data);

                sensor_imu->process(capture_imu);

                D_preint    = processor_imu->getLastPtr()->getDeltaPreint();
                D_corrected = processor_imu->getLastPtr()->getDeltaCorrected(bias_real);
            }
        }

        // Initial configuration of variables
        bool configureAll()
        {
            // variables
            DT      = num_integrations * dt;
            q0      .normalize();
            x0     << p0, q0.coeffs(), v0;
            P0      .setIdentity() * 0.01;
            motion << a, w;

            // wolf objects
            KF_0    = problem->setPrior(x0, P0, t0);
            C_0     = processor_imu->getOriginPtr();

            processor_imu->getLastPtr()->setCalibrationPreint(bias_preint);

            return true;
        }

        // Integrate using all methods
        virtual void integrateAll()
        {
            // ===================================== INTEGRATE EXACTLY WITH IMU_TOOLS with no bias at all
            D_exact  = integrateDelta(num_integrations, q0, motion, bias_null, bias_null, dt);
            x1_exact = imu::composeOverState(x0, D_exact, DT );


            // ===================================== INTEGRATE USING IMU_TOOLS
            // pre-integrate
            D_preint_imu = integrateDelta(num_integrations, q0, motion, bias_real, bias_preint, dt, J_D_bias);

            // correct perturbated
            step             = J_D_bias * (bias_real - bias_preint);
            D_corrected_imu  = imu::plus(D_preint_imu, step);

            // compose states
            x1_preint_imu    = imu::composeOverState(x0, D_preint_imu    , DT );
            x1_corrected_imu = imu::composeOverState(x0, D_corrected_imu , DT );

            // ===================================== INTEGRATE USING PROCESSOR_IMU

            integrateWithProcessor(num_integrations, t0, q0, motion, bias_real, bias_preint, dt, D_preint, D_corrected);

            // compose states
            x1_preint        = imu::composeOverState(x0, D_preint        , DT );
            x1_corrected     = imu::composeOverState(x0, D_corrected     , DT );
        }

        // Set state_blocks status
        void setFixedBlocks()
        {
            // this sets each state block status fixed / unfixed
            KF_0->getPPtr()->setFixed(p0_fixed);
            KF_0->getOPtr()->setFixed(q0_fixed);
            KF_0->getVPtr()->setFixed(v0_fixed);
            KF_1->getPPtr()->setFixed(p1_fixed);
            KF_1->getOPtr()->setFixed(q1_fixed);
            KF_1->getVPtr()->setFixed(v1_fixed);
        }


        void setKfStates()
        {
            // This perturbs states to estimate around the exact value, then assigns to the keyframe

            VectorXs x_pert(10);

            // KF 0
            x_pert = x0;
            if (!p0_fixed)
                x_pert.head(3)      += Vector3s::Random() * 0.01;
            if (!q0_fixed)
                x_pert.segment(3,4) = (Quaternions(x_pert.data() + 3) * exp_q(Vector3s::Random() * 0.01)).coeffs().normalized();
            if (!v0_fixed)
                x_pert.tail(3)      += Vector3s::Random() * 0.01;
            KF_0->setState(x_pert);

            // KF 1
            x_pert = x1_exact;
            if (!p1_fixed)
                x_pert.head(3)      += Vector3s::Random() * 0.01;
            if (!q1_fixed)
                x_pert.segment(3,4) = (Quaternions(x_pert.data() + 3) * exp_q(Vector3s::Random() * 0.01)).coeffs().normalized();
            if (!v1_fixed)
                x_pert.tail(3)      += Vector3s::Random() * 0.01;
            KF_1->setState(x_pert);
        }

        virtual void buildProblem()
        {
            // ===================================== SET KF in Wolf tree
            FrameBasePtr KF = problem->emplaceFrame(KEY_FRAME, x1_exact, t);

            // ===================================== IMU CALLBACK
            processor_imu->keyFrameCallback(KF, 0.01);

            KF_1 = problem->getLastKeyFramePtr();
            C_1  = KF_1->getCaptureList().front(); // front is IMU
            CM_1 = static_pointer_cast<CaptureMotion>(C_1);

            // ===================================== SET BOUNDARY CONDITIONS
            setFixedBlocks();
            setKfStates();
        }

        string solveProblem(int verbose = 1)
        {
            string report = ceres_manager->solve(verbose);

            bias_0 = C_0->getCalibration();
            bias_1 = C_1->getCalibration();

            // ===================================== GET INTEGRATED STATE WITH SOLVED BIAS
            // with processor
            x0_optim     = KF_0->getState();
            D_optim      = CM_1->getDeltaCorrected(bias_0);
            x1_optim     = KF_1->getState();

            // with imu_tools
            step         = J_D_bias * (bias_0 - bias_preint);
            D_optim_imu  = imu::plus(D_preint, step);
            x1_optim_imu = imu::composeOverState(x0, D_optim_imu, DT);

            return report;
        }

        string run(int verbose)
        {
            configureAll();
            integrateAll();
            buildProblem();
            string report = solveProblem(verbose);
            return report;
        }

        void print()
        {
            WOLF_TRACE("D_exact       : ", D_exact            .transpose() ); // exact delta integrated, with absolutely no bias
            WOLF_TRACE("D_preint      : ", D_preint           .transpose() ); // pre-integrated delta using processor
            WOLF_TRACE("D_preint_imu  : ", D_preint_imu       .transpose() ); // pre-integrated delta using imu_tools
            WOLF_TRACE("D_correct     : ", D_corrected        .transpose() ); // corrected delta using processor
            WOLF_TRACE("D_correct_imu : ", D_corrected_imu    .transpose() ); // corrected delta using imu_tools
            WOLF_TRACE("D_optim       : ", D_optim            .transpose() ); // corrected delta using optimum bias after solving using processor
            WOLF_TRACE("D_optim_imu   : ", D_optim_imu        .transpose() ); // corrected delta using optimum bias after solving using imu_tools

            WOLF_TRACE("bias real     : ", bias_real          .transpose() ); // real bias
            WOLF_TRACE("bias preint   : ", bias_preint        .transpose() ); // bias used during pre-integration
            WOLF_TRACE("bias optim 0  : ", bias_0             .transpose() ); // solved bias at KF_0
            WOLF_TRACE("bias optim 1  : ", bias_1             .transpose() ); // solved bias at KF_1

            // states at the end of integration, i.e., at KF_1
            WOLF_TRACE("X1_exact      : ", x1_exact           .transpose() ); // exact state
            WOLF_TRACE("X1_preint     : ", x1_preint          .transpose() ); // state using delta preintegrated by processor
            WOLF_TRACE("X1_preint_imu : ", x1_preint_imu      .transpose() ); // state using delta preintegrated by imu_tools
            WOLF_TRACE("X1_correct    : ", x1_corrected       .transpose() ); // corrected state vy processor
            WOLF_TRACE("X1_correct_imu: ", x1_corrected_imu   .transpose() ); // corrected state by imu_tools
            WOLF_TRACE("X1_optim      : ", x1_optim           .transpose() ); // optimal state after solving using processor
            WOLF_TRACE("X1_optim_imu  : ", x1_optim_imu       .transpose() ); // optimal state after solving using imu_tools
            WOLF_TRACE("err1_optim    : ", (x1_optim-x1_exact).transpose() ); // error of optimal state corrected by imu_tools w.r.t. exact state
            WOLF_TRACE("err1_optim_imu: ", (x1_optim_imu-x1_exact).transpose() ); // error of optimal state corrected by imu_tools w.r.t. exact state
            WOLF_TRACE("X0_optim      : ", x0_optim           .transpose() ); // optimal state after solving using processor
        }


};

class Process_Constraint_IMU_ODO : public Process_Constraint_IMU
{
    public:
        // Wolf objects
        SensorOdom3DPtr     sensor_odo;
        ProcessorOdom3DPtr  processor_odo;

        virtual void SetUp( ) override
        {

            // ===================================== IMU
            Process_Constraint_IMU::SetUp();

            // ===================================== ODO
            string wolf_root = _WOLF_ROOT_DIR;

            SensorBasePtr    sensor     = problem->installSensor   ("ODOM 3D", "Odometer", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D.yaml"   );
            ProcessorBasePtr processor  = problem->installProcessor("ODOM 3D", "Odometer", "Odometer"                            , wolf_root + "/src/examples/processor_odom_3D.yaml");
            sensor_odo      = static_pointer_cast<SensorOdom3D>(sensor);
            processor_odo   = static_pointer_cast<ProcessorOdom3D>(processor);

            // prevent this processor from voting by setting high thresholds :
            processor_odo->setAngleTurned(2.0);
            processor_odo->setDistTraveled(1.0);
            processor_odo->setMaxBuffLength(10);
            processor_odo->setMaxTimeSpan(1.0);
        }

        virtual void integrateAll() override
        {
            // ===================================== IMU
            Process_Constraint_IMU::integrateAll();

            // ===================================== ODO
            Vector6s    data;
            Vector3s    p1  = x1_exact.head(3);
            Quaternions q1   (x1_exact.data() + 3);
            Vector3s    dp  =              q0.conjugate() * (p1 - p0);
            Vector3s    dth = wolf::log_q( q0.conjugate() *  q1     );
            data           << dp, dth;

            CaptureOdom3DPtr capture_odo = make_shared<CaptureOdom3D>(t, sensor_odo, data, sensor_odo->getNoiseCov());

            sensor_odo->process(capture_odo);
       }

        virtual void buildProblem() override
        {
            // ===================================== IMU
            Process_Constraint_IMU::buildProblem();

            // ===================================== ODO
            processor_odo->keyFrameCallback(KF_1, 0.1);
        }

};

TEST_F(Process_Constraint_IMU, Var_B1_B2_Invar_P1_Q1_V1_P2_Q2_V2)
{

    // ================================================================================================================ //
    // ==================================== INITIAL CONDITIONS -- USER OPTIONS ======================================== //
    // ================================================================================================================ //
    //
    // ---------- time
    t0                  = 0;
    dt                  = 0.01;
    num_integrations    = 50;

    // ---------- initial pose
    p0                 << 0,0,0;
    q0.coeffs()        << 0,0,0,1;
    v0                 << 0,0,0;

    // ---------- bias
    bias_real          << .001, .002, .003,    -.001, -.002, -.003;
    bias_preint         = -bias_real;

    // ---------- motion params
    a                  << 1,2,3;
    w                  << 1,2,3;

    // ---------- fix boundaries
    p0_fixed       = true;
    q0_fixed       = true;
    v0_fixed       = true;
    p1_fixed       = true;
    q1_fixed       = true;
    v1_fixed       = true;
    //
    // ===================================== INITIAL CONDITIONS -- USER INPUT ENDS HERE =============================== //
    // ================================================================================================================ //


    // ===================================== RUN ALL
    configureAll();
    integrateAll();
    buildProblem();
    string report = solveProblem(1);
    WOLF_TRACE(report);

    WOLF_TRACE("w * DT (rather be lower than 1.57 approx) = ", w.transpose() * DT); // beware if w*DT is large (>~ 1.57) then Jacobian for correction is poor


    // ===================================== PRINT RESULTS
//    print();


    // ===================================== CHECK ALL (SEE CLASS DEFINITION FOR THE MEANING OF ALL VARIABLES)

    // check delta and state integrals
    ASSERT_MATRIX_APPROX(D_preint           , D_preint_imu      , 1e-8 );
    ASSERT_MATRIX_APPROX(D_corrected        , D_corrected_imu   , 1e-8 );
    ASSERT_MATRIX_APPROX(D_corrected_imu    , D_exact           , 1e-5 );
    ASSERT_MATRIX_APPROX(D_corrected        , D_exact           , 1e-5 );
    ASSERT_MATRIX_APPROX(x1_corrected_imu   , x1_exact          , 1e-5 );
    ASSERT_MATRIX_APPROX(x1_corrected       , x1_exact          , 1e-5 );

    // check optimal solutions
    ASSERT_MATRIX_APPROX(x0_optim           , x0        , 1e-5 );
    ASSERT_NEAR(x0_optim.segment(3,4).norm(), 1.0       , 1e-8 );
    ASSERT_MATRIX_APPROX(bias_0             , bias_real , 1e-4 );
    ASSERT_MATRIX_APPROX(bias_1             , bias_real , 1e-4 );
    ASSERT_MATRIX_APPROX(D_optim            , D_exact   , 1e-5 );
    ASSERT_MATRIX_APPROX(x1_optim           , x1_exact  , 1e-5 );
    ASSERT_MATRIX_APPROX(D_optim_imu        , D_exact   , 1e-5 );
    ASSERT_MATRIX_APPROX(x1_optim_imu       , x1_exact  , 1e-5 );
    ASSERT_NEAR(x1_optim.segment(3,4).norm(), 1.0       , 1e-8 );


}


TEST_F(Process_Constraint_IMU, Var_P1_Q1_V1_B1_B2_Invar_P2_Q2_V2)
{

    // ================================================================================================================ //
    // ==================================== INITIAL CONDITIONS -- USER OPTIONS ======================================== //
    // ================================================================================================================ //
    //
    // ---------- time
    t0                  = 0;
    dt                  = 0.01;
    num_integrations    = 50;

    // ---------- initial pose
    p0                 << 0,0,0;
    q0.coeffs()        << 0,0,0,1;
    v0                 << 0,0,0;

    // ---------- bias
    bias_real          << .001, .002, .003,    -.001, -.002, -.003;
    bias_preint         = -bias_real;

    // ---------- motion params
    a                  << 1,2,3;
    w                  << 1,2,3;

    // ---------- fix boundaries
    p0_fixed       = false;
    q0_fixed       = false;
    v0_fixed       = false;
    p1_fixed       = true;
    q1_fixed       = true;
    v1_fixed       = true;
    //
    // ===================================== INITIAL CONDITIONS -- USER INPUT ENDS HERE =============================== //
    // ================================================================================================================ //


    // ===================================== RUN ALL
    configureAll();
    integrateAll();
    buildProblem();
    string report = solveProblem(1);
    WOLF_TRACE(report);


    // ===================================== PRINT RESULTS
    //    print();


    // ===================================== CHECK ALL (SEE CLASS DEFINITION FOR THE MEANING OF ALL VARIABLES)

    // check delta and state integrals
    ASSERT_MATRIX_APPROX(D_preint           , D_preint_imu      , 1e-8 );
    ASSERT_MATRIX_APPROX(D_corrected        , D_corrected_imu   , 1e-8 );
    ASSERT_MATRIX_APPROX(D_corrected_imu    , D_exact           , 1e-5 );
    ASSERT_MATRIX_APPROX(D_corrected        , D_exact           , 1e-5 );
    ASSERT_MATRIX_APPROX(x1_corrected_imu   , x1_exact          , 1e-5 );
    ASSERT_MATRIX_APPROX(x1_corrected       , x1_exact          , 1e-5 );

    // check optimal solutions
    ASSERT_MATRIX_APPROX(x0_optim           , x0        , 1e-5 );
    ASSERT_NEAR(x0_optim.segment(3,4).norm(), 1.0       , 1e-8 );
    ASSERT_MATRIX_APPROX(bias_0             , bias_real , 1e-4 );
    ASSERT_MATRIX_APPROX(bias_1             , bias_real , 1e-4 );
    ASSERT_MATRIX_APPROX(D_optim            , D_exact   , 1e-5 );
    ASSERT_MATRIX_APPROX(x1_optim           , x1_exact  , 1e-5 );
    ASSERT_MATRIX_APPROX(D_optim_imu        , D_exact   , 1e-5 );
    ASSERT_MATRIX_APPROX(x1_optim_imu       , x1_exact  , 1e-5 );
    ASSERT_NEAR(x1_optim.segment(3,4).norm(), 1.0       , 1e-8 );


}


TEST_F(Process_Constraint_IMU, Var_P1_Q1_B1_V2_B2_Invar_V1_P2_Q2) // PQv_B__pqV_B
{

    // ================================================================================================================ //
    // ==================================== INITIAL CONDITIONS -- USER OPTIONS ======================================== //
    // ================================================================================================================ //
    //
    // ---------- time
    t0                  = 0;
    dt                  = 0.01;
    num_integrations    = 50;

    // ---------- initial pose
    p0                 << 0,0,0;
    q0.coeffs()        << 0,0,0,1;
    v0                 << 0,0,0;

    // ---------- bias
    bias_real          << .001, .002, .003,    -.001, -.002, -.003;
    bias_preint         = -bias_real;

    // ---------- motion params
    a                  << 1,2,3;
    w                  << 1,2,3;

    // ---------- fix boundaries
    p0_fixed       = false;
    q0_fixed       = false;
    v0_fixed       = true;
    p1_fixed       = true;
    q1_fixed       = true;
    v1_fixed       = false;
    //
    // ===================================== INITIAL CONDITIONS -- USER INPUT ENDS HERE =============================== //
    // ================================================================================================================ //


    // ===================================== RUN ALL
    configureAll();
    integrateAll();
    buildProblem();
    string report = solveProblem(1);
    WOLF_TRACE(report);


    // ===================================== PRINT RESULTS
    //    print();


    // ===================================== CHECK ALL (SEE CLASS DEFINITION FOR THE MEANING OF ALL VARIABLES)

    // check delta and state integrals
    ASSERT_MATRIX_APPROX(D_preint           , D_preint_imu      , 1e-8 );
    ASSERT_MATRIX_APPROX(D_corrected        , D_corrected_imu   , 1e-8 );
    ASSERT_MATRIX_APPROX(D_corrected_imu    , D_exact           , 1e-5 );
    ASSERT_MATRIX_APPROX(D_corrected        , D_exact           , 1e-5 );
    ASSERT_MATRIX_APPROX(x1_corrected_imu   , x1_exact          , 1e-5 );
    ASSERT_MATRIX_APPROX(x1_corrected       , x1_exact          , 1e-5 );

    // check optimal solutions
    ASSERT_MATRIX_APPROX(x0_optim           , x0        , 1e-5 );
    ASSERT_NEAR(x0_optim.segment(3,4).norm(), 1.0       , 1e-8 );
    ASSERT_MATRIX_APPROX(bias_0             , bias_real , 1e-4 );
    ASSERT_MATRIX_APPROX(bias_1             , bias_real , 1e-4 );
    ASSERT_MATRIX_APPROX(D_optim            , D_exact   , 1e-5 );
    ASSERT_MATRIX_APPROX(x1_optim           , x1_exact  , 1e-5 );
    ASSERT_MATRIX_APPROX(D_optim_imu        , D_exact   , 1e-5 );
    ASSERT_MATRIX_APPROX(x1_optim_imu       , x1_exact  , 1e-5 );
    ASSERT_NEAR(x1_optim.segment(3,4).norm(), 1.0       , 1e-8 );

}


TEST_F(Process_Constraint_IMU_ODO, Var_P0_Q0_V0_B0_P1_Q1_B1__Invar_V1)
{

    // ================================================================================================================ //
    // ==================================== INITIAL CONDITIONS -- USER OPTIONS ======================================== //
    // ================================================================================================================ //
    //
    // ---------- time
    t0                  = 0;
    dt                  = 0.01;
    num_integrations    = 50;

    // ---------- initial pose
    p0                 << 0,0,0;
    q0.coeffs()        << 0,0,0,1;
    v0                 << 0,0,0;

    // ---------- bias
    bias_real          << .001, .002, .003,    -.001, -.002, -.003;
    bias_preint         = -bias_real;

    // ---------- motion params
    a                  << 1,2,3;
    w                  << 1,2,3;

    // ---------- fix boundaries
    p0_fixed       = false;
    q0_fixed       = false;
    v0_fixed       = false;
    p1_fixed       = false;
    q1_fixed       = false;
    v1_fixed       = true;
    //
    // ===================================== INITIAL CONDITIONS -- USER INPUT ENDS HERE =============================== //
    // ================================================================================================================ //


    // ===================================== RUN ALL
    configureAll();
    integrateAll();
    buildProblem();
    string report = solveProblem(1);
    WOLF_TRACE(report);


    // ===================================== PRINT RESULTS
//    print();


    // ===================================== CHECK ALL (SEE CLASS DEFINITION FOR THE MEANING OF ALL VARIABLES)

    // check delta and state integrals
    ASSERT_MATRIX_APPROX(D_preint           , D_preint_imu      , 1e-8 );
    ASSERT_MATRIX_APPROX(D_corrected        , D_corrected_imu   , 1e-8 );
    ASSERT_MATRIX_APPROX(D_corrected_imu    , D_exact           , 1e-5 );
    ASSERT_MATRIX_APPROX(D_corrected        , D_exact           , 1e-5 );
    ASSERT_MATRIX_APPROX(x1_corrected_imu   , x1_exact          , 1e-5 );
    ASSERT_MATRIX_APPROX(x1_corrected       , x1_exact          , 1e-5 );

    // check optimal solutions
    ASSERT_MATRIX_APPROX(x0_optim           , x0        , 1e-5 );
    ASSERT_NEAR(x0_optim.segment(3,4).norm(), 1.0       , 1e-8 );
    ASSERT_MATRIX_APPROX(bias_0             , bias_real , 1e-4 );
    ASSERT_MATRIX_APPROX(bias_1             , bias_real , 1e-4 );
    ASSERT_MATRIX_APPROX(D_optim            , D_exact   , 1e-5 );
    ASSERT_MATRIX_APPROX(x1_optim           , x1_exact  , 1e-5 );
    ASSERT_MATRIX_APPROX(D_optim_imu        , D_exact   , 1e-5 );
    ASSERT_MATRIX_APPROX(x1_optim_imu       , x1_exact  , 1e-5 );
    ASSERT_NEAR(x1_optim.segment(3,4).norm(), 1.0       , 1e-8 );
}


TEST_F(Process_Constraint_IMU_ODO, Var_P0_Q0_B0_P1_Q1_V1_B1__Invar_V0)
{

    // ================================================================================================================ //
    // ==================================== INITIAL CONDITIONS -- USER OPTIONS ======================================== //
    // ================================================================================================================ //
    //
    // ---------- time
    t0                  = 0;
    dt                  = 0.01;
    num_integrations    = 50;

    // ---------- initial pose
    p0                 << 0,0,0;
    q0.coeffs()        << 0,0,0,1;
    v0                 << 0,0,0;

    // ---------- bias
    bias_real          << .001, .002, .003,    -.001, -.002, -.003;
    bias_preint         = -bias_real;

    // ---------- motion params
    a                  << 1,2,3;
    w                  << 1,2,3;

    // ---------- fix boundaries
    p0_fixed       = false;
    q0_fixed       = false;
    v0_fixed       = true;
    p1_fixed       = false;
    q1_fixed       = false;
    v1_fixed       = false;
    //
    // ===================================== INITIAL CONDITIONS -- USER INPUT ENDS HERE =============================== //
    // ================================================================================================================ //


    // ===================================== RUN ALL
    configureAll();
    integrateAll();
    buildProblem();
    string report = solveProblem(1);
    WOLF_TRACE(report);


    // ===================================== PRINT RESULTS
//    print();


    // ===================================== CHECK ALL (SEE CLASS DEFINITION FOR THE MEANING OF ALL VARIABLES)

    // check delta and state integrals
    ASSERT_MATRIX_APPROX(D_preint           , D_preint_imu      , 1e-8 );
    ASSERT_MATRIX_APPROX(D_corrected        , D_corrected_imu   , 1e-8 );
    ASSERT_MATRIX_APPROX(D_corrected_imu    , D_exact           , 1e-5 );
    ASSERT_MATRIX_APPROX(D_corrected        , D_exact           , 1e-5 );
    ASSERT_MATRIX_APPROX(x1_corrected_imu   , x1_exact          , 1e-5 );
    ASSERT_MATRIX_APPROX(x1_corrected       , x1_exact          , 1e-5 );

    // check optimal solutions
    ASSERT_MATRIX_APPROX(x0_optim           , x0        , 1e-5 );
    ASSERT_NEAR(x0_optim.segment(3,4).norm(), 1.0       , 1e-8 );
    ASSERT_MATRIX_APPROX(bias_0             , bias_real , 1e-4 );
    ASSERT_MATRIX_APPROX(bias_1             , bias_real , 1e-4 );
    ASSERT_MATRIX_APPROX(D_optim            , D_exact   , 1e-5 );
    ASSERT_MATRIX_APPROX(x1_optim           , x1_exact  , 1e-5 );
    ASSERT_MATRIX_APPROX(D_optim_imu        , D_exact   , 1e-5 );
    ASSERT_MATRIX_APPROX(x1_optim_imu       , x1_exact  , 1e-5 );
    ASSERT_NEAR(x1_optim.segment(3,4).norm(), 1.0       , 1e-8 );
}



int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
//        ::testing::GTEST_FLAG(filter) = "Process_Constraint_IMU.*";
//    ::testing::GTEST_FLAG(filter) = "Process_Constraint_IMU_ODO.*";

    return RUN_ALL_TESTS();
}

