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
        CeresManagerPtr     ceres_manager;
        SensorIMUPtr        sensor_imu;
        ProcessorIMUPtr     processor_imu;
        CaptureIMUPtr       capture_imu;
        FrameBasePtr        KF_0, KF_1;     // keyframes
        CaptureBasePtr      C_0 , C_1;      // base captures
        CaptureMotionPtr    CM_0, CM_1;     // motion captures

        // time
        TimeStamp           t0, t;
        Scalar              dt, DT;
        int                 num_integrations;

        // initial state
        VectorXs            x0;                                 // initial state
        Vector3s            p0, v0;                             // initial pos and vel
        Quaternions         q0, q;                              // initial and current orientations
        Matrix<Scalar,9,9>  P0;                                 // initial state covariance

        // bias
        Vector6s            bias_real, bias_preint, bias_null;  // real, pre-int and zero biases.
        Vector6s            bias_0, bias_1;                     // optimized bias at KF's 0 and 1

        // input
        Matrix<Scalar, 6, Dynamic> motion;                      // Motion in IMU frame. Each column is a motion step. If just one column, then the number of steps is defined in num_integrations
        Matrix<Scalar, 3, Dynamic> a, w;                        // True acc and angvel in IMU frame. Each column is a motion step. Used to create motion with `motion << a,w;`
        Vector6s            data;                               // IMU data. It's the motion with gravity and bias. See imu::motion2data().

        // Deltas and states (exact, integrated, corrected, solved, etc)
        VectorXs        D_exact,         x1_exact;          // exact or ground truth
        VectorXs        D_preint_imu,    x1_preint_imu;     // preintegrated with imu_tools
        VectorXs        D_corrected_imu, x1_corrected_imu;  // corrected with imu_tools
        VectorXs        D_preint,        x1_preint;         // preintegrated with processor_imu
        VectorXs        D_corrected,     x1_corrected;      // corrected with processor_imu
        VectorXs        D_optim,         x1_optim;          // optimized using constraint_imu
        VectorXs        D_optim_imu,     x1_optim_imu;      // corrected with imu_tools using optimized bias
        VectorXs                         x0_optim;          // optimized using constraint_imu

        // Trajectory buffer of correction Jacobians
        std::vector<MatrixXs> Buf_Jac_preint_prc;

        // Trajectory matrices
        MatrixXs Trj_D_exact, Trj_D_preint_imu, Trj_D_preint_prc, Trj_D_corrected_imu, Trj_D_corrected_prc;
        MatrixXs Trj_x_exact, Trj_x_preint_imu, Trj_x_preint_prc, Trj_x_corrected_imu, Trj_x_corrected_prc;

        // Delta correction Jacobian and step
        Matrix<Scalar,9,6>  J_D_bias;                           // Jacobian of pre-integrated delta w
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

            dt = 0.01;
            processor_imu->setTimeTolerance(dt/2);

            // Some initializations
            bias_null   .setZero();
            x0          .resize(10);
            D_preint    .resize(10);
            D_corrected .resize(10);
            x1_optim    .resize(10);
            x1_optim_imu.resize(10);

        }


        
        /* Integrate one step of acc and angVel motion, obtain Delta_preintegrated
         * Input:
         *   q: current orientation
         *   motion: [ax, ay, az, wx, wy, wz] as the true magnitudes in body frame
         *   bias_real: the real bias of the IMU
         *   bias_preint: the bias used for Delta pre-integration
         * Input/output
         *   Delta: the preintegrated delta
         *   J_D_b_ptr: a pointer to the Jacobian of Delta wrt bias. Defaults to nullptr.
         */
        static void integrateOneStep(const VectorXs& motion, const VectorXs& bias_real, const VectorXs& bias_preint, Scalar dt, Quaternions& q_real, VectorXs& Delta, Matrix<Scalar, 9, 6>* J_D_b_ptr = nullptr)
        {
            VectorXs delta(10), data(6);
            VectorXs Delta_plus(10);
            Matrix<Scalar, 9, 6> J_d_d, J_D_b, J_d_b;
            Matrix<Scalar, 9, 9> J_D_D, J_D_d;

            data                = imu::motion2data(motion, q_real, bias_real);
            q_real              = q_real*exp_q(motion.tail(3)*dt);
            Vector6s body       = data - bias_preint;
            if (J_D_b_ptr == nullptr)
            {
                delta           = imu::body2delta(body, dt);
                Delta_plus      = imu::compose(Delta, delta, dt);
            }
            else
            {
                imu::body2delta(body, dt, delta, J_d_d);
                imu::compose(Delta, delta, dt, Delta_plus, J_D_D, J_D_d);
                J_D_b           = *J_D_b_ptr;
                J_d_b           = - J_d_d;
                J_D_b           = J_D_D*J_D_b + J_D_d*J_d_b;
                *J_D_b_ptr      = J_D_b;
            }
            Delta               = Delta_plus;
        }


        /* Integrate acc and angVel motion, obtain Delta_preintegrated
         * Input:
         *   N: number of steps
         *   q0: initial orientation
         *   motion: [ax, ay, az, wx, wy, wz] as the true magnitudes in body frame
         *   bias_real: the real bias of the IMU
         *   bias_preint: the bias used for Delta pre-integration
         * Output:
         *   return: the preintegrated delta
         */
        static VectorXs integrateDelta(int N, const Quaternions& q0, const VectorXs& motion, const VectorXs& bias_real, const VectorXs& bias_preint, Scalar dt)
        {
            VectorXs    Delta(10);
            Delta       = imu::identity();
            Quaternions q(q0);
            for (int n = 0; n < N; n++)
            {
                integrateOneStep(motion, bias_real, bias_preint, dt, q, Delta);
            }
            return Delta;
        }

        /* Integrate acc and angVel motion, obtain Delta_preintegrated
         * Input:
         *   N: number of steps
         *   q0: initial orientation
         *   motion: [ax, ay, az, wx, wy, wz] as the true magnitudes in body frame
         *   bias_real: the real bias of the IMU
         *   bias_preint: the bias used for Delta pre-integration
         * Output:
         *   J_D_b: the Jacobian of the preintegrated delta wrt the bias
         *   return: the preintegrated delta
         */
        static VectorXs integrateDelta(int N, const Quaternions& q0, const VectorXs& motion, const VectorXs& bias_real, const VectorXs& bias_preint, Scalar dt, Matrix<Scalar, 9, 6>& J_D_b)
        {
            VectorXs    Delta(10);
            Quaternions q;

            Delta   = imu::identity();
            J_D_b   .setZero();
            q       = q0;
            for (int n = 0; n < N; n++)
            {
                integrateOneStep(motion, bias_real, bias_preint, dt, q, Delta, &J_D_b);
            }
            return Delta;
        }

        /* Integrate acc and angVel motion, obtain Delta_preintegrated
         * Input:
         *   q0: initial orientation
         *   motion: Matrix with N columns [ax, ay, az, wx, wy, wz] with the true magnitudes in body frame
         *   bias_real: the real bias of the IMU
         *   bias_preint: the bias used for Delta pre-integration
         * Output:
         *   J_D_b: the Jacobian of the preintegrated delta wrt the bias
         *   return: the preintegrated delta
         */
        static VectorXs integrateDelta(const Quaternions& q0, const MatrixXs& motion, const VectorXs& bias_real, const VectorXs& bias_preint, Scalar dt, Matrix<Scalar, 9, 6>& J_D_b)
        {
            VectorXs    Delta(10);
            Quaternions q;

            Delta   = imu::identity();
            J_D_b   .setZero();
            q       = q0;
            for (int n = 0; n < motion.cols(); n++)
            {
                integrateOneStep(motion.col(n), bias_real, bias_preint, dt, q, Delta, &J_D_b);
            }
            return Delta;
        }

        /* Integrate acc and angVel motion, obtain Delta_preintegrated
         * Input:
         *   q0: initial orientation
         *   motion: Matrix with N columns [ax, ay, az, wx, wy, wz] with the true magnitudes in body frame
         *   bias_real: the real bias of the IMU
         *   bias_preint: the bias used for Delta pre-integration
         * Output:
         *   J_D_b: the Jacobian of the preintegrated delta wrt the bias
         *   return: the preintegrated delta
         */
        static MotionBuffer integrateDeltaTrajectory(const Quaternions& q0, const MatrixXs& motion, const VectorXs& bias_real, const VectorXs& bias_preint, Scalar dt, Matrix<Scalar, 9, 6>& J_D_b)
        {
            MotionBuffer trajectory(6, 10, 9, 6);
            VectorXs    Delta(10);
            MatrixXs    M9(9,9), M6(6,6), J9(9,9), J96(9,6), V10(10,1), V6(6,1);
            Quaternions q;

            Delta   = imu::identity();
            J_D_b   .setZero();
            q       = q0;
            TimeStamp t(0);
            trajectory.get().emplace_back(t, Vector6s::Zero(), M6, VectorXs::Zero(10), M9, imu::identity(), M9, J9, J9, MatrixXs::Zero(9,6));
            for (int n = 0; n < motion.cols(); n++)
            {
                t += dt;
                integrateOneStep(motion.col(n), bias_real, bias_preint, dt, q, Delta, &J_D_b);
                trajectory.get().emplace_back(t, motion.col(n), M6, V10, M9, Delta, M9, J9, J9, J_D_b);
            }
            return trajectory;
        }



        MotionBuffer integrateWithProcessor(int N, const TimeStamp& t0, const Quaternions q0, const MatrixXs& motion, const VectorXs& bias_real, const VectorXs& bias_preint, Scalar dt, VectorXs& D_preint, VectorXs& D_corrected)
        {
            Vector6s      motion_now;
            data        = imu::motion2data(motion.col(0), q0, bias_real);
            capture_imu = make_shared<CaptureIMU>(t0, sensor_imu, data, sensor_imu->getNoiseCov());
            q           = q0;
            t           = t0;
            for (int i= 0; i < N; i++)
            {
                t   += dt;
                motion_now = motion.cols() == 1
                                ? motion
                                : motion.col(i);
                data = imu::motion2data(motion_now, q, bias_real);
                w    = motion_now.tail<3>();
                q    = q * exp_q(w*dt);

                capture_imu->setTimeStamp(t);
                capture_imu->setData(data);

                sensor_imu->process(capture_imu);

                D_preint    = processor_imu->getLastPtr()->getDeltaPreint();
                D_corrected = processor_imu->getLastPtr()->getDeltaCorrected(bias_real);
            }
            return processor_imu->getBuffer();
        }



        // Initial configuration of variables
        bool configureAll()
        {
            // initial state
            q0      .normalize();
            x0     << p0, q0.coeffs(), v0;
            P0      .setIdentity() * 0.01;

            // motion
            if (motion.cols() == 0)
            {
                motion.resize(6,a.cols());
                motion << a, w;
            }
            else
            {
                // if motion has any column at all, then it is already initialized in TEST_F(...) and we do nothing.
            }
            if (motion.cols() != 1)
            {
                // if motion has more than 1 col, make num_integrations consistent with nbr of cols, just for consistency
                num_integrations = motion.cols();
            }

            // total run time
            DT      = num_integrations * dt;

            // wolf objects
            KF_0    = problem->setPrior(x0, P0, t0, dt/2);
            C_0     = processor_imu->getOriginPtr();

            processor_imu->getLastPtr()->setCalibrationPreint(bias_preint);

            return true;
        }



        // Integrate using all methods
        virtual void integrateAll()
        {
            // ===================================== INTEGRATE EXACTLY WITH IMU_TOOLS with no bias at all
            if (motion.cols() == 1)
                D_exact = integrateDelta(num_integrations, q0, motion, bias_null, bias_null, dt);
            else
                D_exact = integrateDelta(q0, motion, bias_null, bias_null, dt, J_D_bias);
            x1_exact = imu::composeOverState(x0, D_exact, DT );


            // ===================================== INTEGRATE USING IMU_TOOLS
            // pre-integrate
            if (motion.cols() == 1)
                D_preint_imu = integrateDelta(num_integrations, q0, motion, bias_real, bias_preint, dt, J_D_bias);
            else
                D_preint_imu = integrateDelta(q0, motion, bias_real, bias_preint, dt, J_D_bias);

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


        // Integrate Trajectories all methods
        virtual void integrateAllTrajectories()
        {
            Size cols = motion.cols() + 1;
            Trj_D_exact.resize(10,cols); Trj_D_preint_imu.resize(10,cols); Trj_D_preint_prc.resize(10,cols); Trj_D_corrected_imu.resize(10,cols); Trj_D_corrected_prc.resize(10,cols);
            Trj_x_exact.resize(10,cols); Trj_x_preint_imu.resize(10,cols); Trj_x_preint_prc.resize(10,cols); Trj_x_corrected_imu.resize(10,cols); Trj_x_corrected_prc.resize(10,cols);


            // ===================================== INTEGRATE EXACTLY WITH IMU_TOOLS with no bias at all
            MotionBuffer Buf_exact = integrateDeltaTrajectory(q0, motion, bias_null, bias_null, dt, J_D_bias);

            // Build exact trajectories
            int col = 0;
            Scalar Dt = 0;
            for (auto m : Buf_exact.get())
            {
                Trj_D_exact.col(col) = m.delta_integr_;
                Trj_x_exact.col(col) = imu::composeOverState(x0, m.delta_integr_, Dt );
                Dt += dt;
                col ++;
            }

            // set
            D_exact          = Trj_D_exact.col(cols-1);
            x1_exact         = Trj_x_exact.col(cols-1);

            // ===================================== INTEGRATE USING IMU_TOOLS
            // pre-integrate
            MotionBuffer Buf_preint_imu = integrateDeltaTrajectory(q0, motion, bias_real, bias_preint, dt, J_D_bias);

            // Build trajectories preintegrated and corrected with imu_tools
            col = 0;
            Dt = 0;
            for (auto m : Buf_preint_imu.get())
            {
                // preint
                Trj_D_preint_imu.col(col) = m.delta_integr_;
                Trj_x_preint_imu.col(col) = imu::composeOverState(x0, Trj_D_preint_imu.col(col).eval(), Dt );

                // corrected
                VectorXs step                = m.jacobian_calib_ * (bias_real - bias_preint);
                Trj_D_corrected_imu.col(col) = imu::plus(m.delta_integr_, step) ;
                Trj_x_corrected_imu.col(col) = imu::composeOverState(x0, Trj_D_corrected_imu.col(col).eval(), Dt );
                Dt += dt;
                col ++;
            }

            D_preint_imu     = Trj_D_preint_imu.col(cols-1);

            // correct perturbated
            step             = J_D_bias * (bias_real - bias_preint);
            D_corrected_imu  = imu::plus(D_preint_imu, step);

            // compose states
            x1_preint_imu    = imu::composeOverState(x0, D_preint_imu    , DT );
            x1_corrected_imu = imu::composeOverState(x0, D_corrected_imu , DT );

            // ===================================== INTEGRATE USING PROCESSOR_IMU

            MotionBuffer Buf_D_preint_prc = integrateWithProcessor(num_integrations, t0, q0, motion, bias_real, bias_preint, dt, D_preint, D_corrected);


            // Build trajectories preintegrated and corrected with processorIMU
            Dt = 0;
            col = 0;
            Buf_Jac_preint_prc.clear();
            for (auto m : Buf_D_preint_prc.get())
            {
                // preint
                Trj_D_preint_prc.col(col) = m.delta_integr_;
                Trj_x_preint_prc.col(col) = imu::composeOverState(x0, Trj_D_preint_prc.col(col).eval(), Dt );

                // corrected
                VectorXs step                = m.jacobian_calib_ * (bias_real - bias_preint);
                Trj_D_corrected_prc.col(col) = imu::plus(m.delta_integr_, step) ;
                Trj_x_corrected_prc.col(col) = imu::composeOverState(x0, Trj_D_corrected_prc.col(col).eval(), Dt );

                Buf_Jac_preint_prc.push_back(m.jacobian_calib_);
                Dt += dt;
                col ++;
            }

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
            // Perturbations are applied only if the state block is unfixed

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
            problem->keyFrameCallback(KF, nullptr, dt/2);

            // Process IMU for the callback to take effect
            data = Vector6s::Zero();
            capture_imu = make_shared<CaptureIMU>(t+dt, sensor_imu, data, sensor_imu->getNoiseCov());
            processor_imu->process(capture_imu);

            KF_1 = problem->getLastKeyFramePtr();
            C_1  = KF_1->getCaptureList().front(); // front is IMU
            CM_1 = static_pointer_cast<CaptureMotion>(C_1);

            // ===================================== SET BOUNDARY CONDITIONS
            setFixedBlocks();
            setKfStates();
        }



        string solveProblem(int verbose = 1)
        {
            string report   = ceres_manager->solve(verbose);

            bias_0          = C_0->getCalibration();
            bias_1          = C_1->getCalibration();

            // ===================================== GET INTEGRATED STATE WITH SOLVED BIAS
            // with processor
            x0_optim        = KF_0->getState();
            D_optim         = CM_1->getDeltaCorrected(bias_0);
            x1_optim        = KF_1->getState();

            // with imu_tools
            step            = J_D_bias * (bias_0 - bias_preint);
            D_optim_imu     = imu::plus(D_preint, step);
            x1_optim_imu    = imu::composeOverState(x0_optim, D_optim_imu, DT);

            return report;
        }



        string runAll(int verbose)
        {
            configureAll();
            integrateAll();
            buildProblem();
            string report = solveProblem(verbose);
            return report;
        }



        void printAll(std::string report = "")
        {
            WOLF_TRACE(report);

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



        virtual void assertAll()
        {
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

};

class Process_Constraint_IMU_ODO : public Process_Constraint_IMU
{
    public:
        // Wolf objects
        SensorOdom3DPtr     sensor_odo;
        ProcessorOdom3DPtr  processor_odo;
        CaptureOdom3DPtr    capture_odo;

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
            processor_odo->setTimeTolerance(dt/2);

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
            Vector3s    dp  =        q0.conjugate() * (p1 - p0);
            Vector3s    dth = log_q( q0.conjugate() *  q1     );
            data           << dp, dth;

            CaptureOdom3DPtr capture_odo = make_shared<CaptureOdom3D>(t, sensor_odo, data, sensor_odo->getNoiseCov());

            sensor_odo->process(capture_odo);
       }

        virtual void integrateAllTrajectories() override
        {
            // ===================================== IMU
            Process_Constraint_IMU::integrateAllTrajectories();

            // ===================================== ODO
            Vector6s    data;
            Vector3s    p1  = x1_exact.head(3);
            Quaternions q1   (x1_exact.data() + 3);
            Vector3s    dp  =        q0.conjugate() * (p1 - p0);
            Vector3s    dth = log_q( q0.conjugate() *  q1     );
            data           << dp, dth;

            CaptureOdom3DPtr capture_odo = make_shared<CaptureOdom3D>(t, sensor_odo, data, sensor_odo->getNoiseCov());

            sensor_odo->process(capture_odo);
       }

        virtual void buildProblem() override
        {
            // ===================================== IMU
            Process_Constraint_IMU::buildProblem();

            // ===================================== ODO
            // Process ODO for the callback to take effect
            data = Vector6s::Zero();
            capture_odo = make_shared<CaptureOdom3D>(t+dt, sensor_odo, data, sensor_odo->getNoiseCov());
            processor_odo->process(capture_odo);

        }

};

TEST_F(Process_Constraint_IMU, MotionConstant_PQV_b__PQV_b) // F_ixed___e_stimated
{

    // ================================================================================================================ //
    // ==================================== INITIAL CONDITIONS -- USER OPTIONS ======================================== //
    // ================================================================================================================ //
    //
    // ---------- time
    t0                  = 0;
    num_integrations    = 50;

    // ---------- initial pose
    p0                 << 0,0,0;
    q0.coeffs()        << 0,0,0,1;
    v0                 << 0,0,0;

    // ---------- bias
    bias_real          << .001, .002, .003,    -.001, -.002, -.003;
    bias_preint         = -bias_real;

    // ---------- motion params
    a                  = Vector3s( 1,2,3 );
    w                  = Vector3s( 1,2,3 );

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
    string report = runAll(1);

//    printAll(report);

    assertAll();

}

TEST_F(Process_Constraint_IMU, test_capture) // F_ixed___e_stimated
{

    // ================================================================================================================ //
    // ==================================== INITIAL CONDITIONS -- USER OPTIONS ======================================== //
    // ================================================================================================================ //
    //
    // ---------- time
    t0                  = 0;
    num_integrations    = 50;

    // ---------- initial pose
    p0                 << 0,0,0;
    q0.coeffs()        << 0,0,0,1;
    v0                 << 0,0,0;

    // ---------- bias
    bias_real          << .001, .002, .003,    -.001, -.002, -.003;
    bias_preint         = -bias_real;

    // ---------- motion params
    a                  = Vector3s( 1,2,3 );
    w                  = Vector3s( 1,2,3 );

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
    Eigen::Vector6s initial_bias((Eigen::Vector6s()<< .002, .0007, -.001,   .003, -.002, .001).finished());
    sensor_imu->getIntrinsicPtr()->setState(initial_bias);
    configureAll();
    integrateAll();
    buildProblem();
    //Since we did not solve, hence bias estimates did not change yet, both capture should have the same calibration
    ASSERT_MATRIX_APPROX(KF_0->getCaptureOf(sensor_imu)->getCalibration(), initial_bias, 1e-8 );
    ASSERT_MATRIX_APPROX(KF_0->getCaptureOf(sensor_imu)->getCalibration(), KF_1->getCaptureOf(sensor_imu)->getCalibration(), 1e-8 );
}


TEST_F(Process_Constraint_IMU, MotionConstant_pqv_b__PQV_b) // F_ixed___e_stimated
{

    // ================================================================================================================ //
    // ==================================== INITIAL CONDITIONS -- USER OPTIONS ======================================== //
    // ================================================================================================================ //
    //
    // ---------- time
    t0                  = 0;
    num_integrations    = 50;

    // ---------- initial pose
    p0                 << 0,0,0;
    q0.coeffs()        << 0,0,0,1;
    v0                 << 0,0,0;

    // ---------- bias
    bias_real          << .001, .002, .003,    -.001, -.002, -.003;
    bias_preint         = -bias_real;

    // ---------- motion params
    a                  = Vector3s( 1,2,3 );
    w                  = Vector3s( 1,2,3 );

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
    string report = runAll(1);

    //    printAll(report);

    assertAll();

}


TEST_F(Process_Constraint_IMU, MotionConstant_pqV_b__PQv_b) // F_ixed___e_stimated
{

    // ================================================================================================================ //
    // ==================================== INITIAL CONDITIONS -- USER OPTIONS ======================================== //
    // ================================================================================================================ //
    //
    // ---------- time
    t0                  = 0;
    num_integrations    = 50;

    // ---------- initial pose
    p0                 << 0,0,0;
    q0.coeffs()        << 0,0,0,1;
    v0                 << 0,0,0;

    // ---------- bias
    bias_real          << .001, .002, .003,    -.001, -.002, -.003;
    bias_preint         = -bias_real;

    // ---------- motion params
    a                  = Vector3s( 1,2,3 );
    w                  = Vector3s( 1,2,3 );

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
    string report = runAll(1);

    //    printAll(report);

    assertAll();

}


TEST_F(Process_Constraint_IMU, MotionRandom_PQV_b__PQV_b) // F_ixed___e_stimated
{

    // ================================================================================================================ //
    // ==================================== INITIAL CONDITIONS -- USER OPTIONS ======================================== //
    // ================================================================================================================ //
    //
    // ---------- time
    t0                  = 0;
    num_integrations    = 50;

    // ---------- initial pose
    p0                 << 0,0,0;
    q0.coeffs()        << 0,0,0,1;
    v0                 << 0,0,0;

    // ---------- bias
    bias_real          << .001, .002, .003,    -.001, -.002, -.003;
    bias_preint         = -bias_real;

    // ---------- motion params
    a                  = Matrix<Scalar, 3, 50>::Random();
    w                  = Matrix<Scalar, 3, 50>::Random();

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
    string report = runAll(1);

//    printAll(report);

    assertAll();

}


TEST_F(Process_Constraint_IMU, MotionRandom_pqV_b__PQv_b) // F_ixed___e_stimated
{

    // ================================================================================================================ //
    // ==================================== INITIAL CONDITIONS -- USER OPTIONS ======================================== //
    // ================================================================================================================ //
    //
    // ---------- time
    t0                  = 0;
    num_integrations    = 50;

    // ---------- initial pose
    p0                 << 0,0,0;
    q0.coeffs()        << 0,0,0,1;
    v0                 << 0,0,0;

    // ---------- bias
    bias_real          << .001, .002, .003,    -.001, -.002, -.003;
    bias_preint         = -bias_real;

    // ---------- motion params
    a                  = Matrix<Scalar, 3, 50>::Random();
    w                  = Matrix<Scalar, 3, 50>::Random();

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
    string report = runAll(1);

//    printAll(report);

    assertAll();

}

TEST_F(Process_Constraint_IMU, MotionRandom_pqV_b__pQV_b) // F_ixed___e_stimated
{

    // ================================================================================================================ //
    // ==================================== INITIAL CONDITIONS -- USER OPTIONS ======================================== //
    // ================================================================================================================ //
    //
    // ---------- time
    t0                  = 0;
    num_integrations    = 50;

    // ---------- initial pose
    p0                 << 0,0,0;
    q0.coeffs()        << 0,0,0,1;
    v0                 << 0,0,0;

    // ---------- bias
    bias_real          << .001, .002, .003,    -.001, -.002, -.003;
    bias_preint         = -bias_real;

    // ---------- motion params
    a                  = Matrix<Scalar, 3, 50>::Random();
    w                  = Matrix<Scalar, 3, 50>::Random();

    // ---------- fix boundaries
    p0_fixed       = false;
    q0_fixed       = false;
    v0_fixed       = true;
    p1_fixed       = false;
    q1_fixed       = true;
    v1_fixed       = true;
    //
    // ===================================== INITIAL CONDITIONS -- USER INPUT ENDS HERE =============================== //
    // ================================================================================================================ //


    // ===================================== RUN ALL
    string report = runAll(1);

    // printAll(report);

    assertAll();

}

TEST_F(Process_Constraint_IMU, MotionConstant_NonNullState_PQV_b__PQV_b) // F_ixed___e_stimated
{

    // ================================================================================================================ //
    // ==================================== INITIAL CONDITIONS -- USER OPTIONS ======================================== //
    // ================================================================================================================ //
    //
    // ---------- time
    t0                  = 0;
    num_integrations    = 50;

    // ---------- initial pose
    p0                 << 3,2,1;
    q0.coeffs()        << 0.5,0.5,0.5,.5;
    v0                 << 1,2,3;

    // ---------- bias
    bias_real          << .001, .002, .003,    -.001, -.002, -.003;
    bias_preint         = -bias_real;

    // ---------- motion params
    a                  = Vector3s( 1,2,3 );
    w                  = Vector3s( 1,2,3 );

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
    string report = runAll(1);

//    printAll(report);

    assertAll();

}

TEST_F(Process_Constraint_IMU_ODO, MotionConstantRotation_PQV_b__PQV_b) // F_ixed___e_stimated
{

    // ================================================================================================================ //
    // ==================================== INITIAL CONDITIONS -- USER OPTIONS ======================================== //
    // ================================================================================================================ //
    //
    // ---------- time
    t0                  = 0;
    num_integrations    = 50;

    // ---------- initial pose
    p0                 << 0,0,0;
    q0.coeffs()        << 0,0,0,1;
    v0                 << 0,0,0;

    // ---------- bias
    bias_real          << .001, .002, .003,    -.001, -.002, -.003;
    bias_preint         = -bias_real;

    // ---------- motion params
    a                  = Vector3s( 0,0,0 );
    w                  = Vector3s( 1,2,3 );

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
    string report = runAll(1);

    //    printAll(report);

    assertAll();

}

TEST_F(Process_Constraint_IMU_ODO, MotionConstantRotation_PQV_b__PQv_b) // F_ixed___e_stimated
{

    // ================================================================================================================ //
    // ==================================== INITIAL CONDITIONS -- USER OPTIONS ======================================== //
    // ================================================================================================================ //
    //
    // ---------- time
    t0                  = 0;
    num_integrations    = 50;

    // ---------- initial pose
    p0                 << 0,0,0;
    q0.coeffs()        << 0,0,0,1;
    v0                 << 0,0,0;

    // ---------- bias
    bias_real          << .001, .002, .003,    -.001, -.002, -.003;
    bias_preint         = -bias_real;

    // ---------- motion params
    a                  = Vector3s( 0,0,0 );
    w                  = Vector3s( 1,2,3 );

    // ---------- fix boundaries
    p0_fixed       = true;
    q0_fixed       = true;
    v0_fixed       = true;
    p1_fixed       = true;
    q1_fixed       = true;
    v1_fixed       = false;
    //
    // ===================================== INITIAL CONDITIONS -- USER INPUT ENDS HERE =============================== //
    // ================================================================================================================ //


    // ===================================== RUN ALL
    string report = runAll(1);

    //    printAll(report);

    assertAll();

}

TEST_F(Process_Constraint_IMU_ODO, MotionConstantRotation_PQV_b__Pqv_b) // F_ixed___e_stimated
{

    // ================================================================================================================ //
    // ==================================== INITIAL CONDITIONS -- USER OPTIONS ======================================== //
    // ================================================================================================================ //
    //
    // ---------- time
    t0                  = 0;
    num_integrations    = 50;

    // ---------- initial pose
    p0                 << 0,0,0;
    q0.coeffs()        << 0,0,0,1;
    v0                 << 0,0,0;

    // ---------- bias
    bias_real          << .001, .002, .003,    -.001, -.002, -.003;
    bias_preint         = -bias_real;

    // ---------- motion params
    a                  = Vector3s( 0,0,0 );
    w                  = Vector3s( 1,2,3 );

    // ---------- fix boundaries
    p0_fixed       = true;
    q0_fixed       = true;
    v0_fixed       = true;
    p1_fixed       = true;
    q1_fixed       = false;
    v1_fixed       = false;
    //
    // ===================================== INITIAL CONDITIONS -- USER INPUT ENDS HERE =============================== //
    // ================================================================================================================ //


    // ===================================== RUN ALL
    string report = runAll(1);

    // printAll(report);

    assertAll();

}

TEST_F(Process_Constraint_IMU_ODO, MotionConstantRotation_PQV_b__pQv_b) // F_ixed___e_stimated
{

    // ================================================================================================================ //
    // ==================================== INITIAL CONDITIONS -- USER OPTIONS ======================================== //
    // ================================================================================================================ //
    //
    // ---------- time
    t0                  = 0;
    num_integrations    = 50;

    // ---------- initial pose
    p0                 << 0,0,0;
    q0.coeffs()        << 0,0,0,1;
    v0                 << 0,0,0;

    // ---------- bias
    bias_real          << .001, .002, .003,    -.001, -.002, -.003;
    bias_preint         = -bias_real;

    // ---------- motion params
    a                  = Vector3s( 0,0,0 );
    w                  = Vector3s( 1,2,3 );

    // ---------- fix boundaries
    p0_fixed       = true;
    q0_fixed       = true;
    v0_fixed       = true;
    p1_fixed       = false;
    q1_fixed       = true;
    v1_fixed       = false;
    //
    // ===================================== INITIAL CONDITIONS -- USER INPUT ENDS HERE =============================== //
    // ================================================================================================================ //


    // ===================================== RUN ALL
    string report = runAll(1);

    //    printAll(report);

    assertAll();

}

TEST_F(Process_Constraint_IMU_ODO, MotionConstantRotation_PQV_b__pqv_b) // F_ixed___e_stimated
{

    // ================================================================================================================ //
    // ==================================== INITIAL CONDITIONS -- USER OPTIONS ======================================== //
    // ================================================================================================================ //
    //
    // ---------- time
    t0                  = 0;
    num_integrations    = 50;

    // ---------- initial pose
    p0                 << 0,0,0;
    q0.coeffs()        << 0,0,0,1;
    v0                 << 0,0,0;

    // ---------- bias
    bias_real          << .001, .002, .003,    -.001, -.002, -.003;
    bias_preint         = -bias_real;

    // ---------- motion params
    a                  = Vector3s( 0,0,0 );
    w                  = Vector3s( 1,2,3 );

    // ---------- fix boundaries
    p0_fixed       = true;
    q0_fixed       = true;
    v0_fixed       = true;
    p1_fixed       = false;
    q1_fixed       = false;
    v1_fixed       = false;
    //
    // ===================================== INITIAL CONDITIONS -- USER INPUT ENDS HERE =============================== //
    // ================================================================================================================ //


    // ===================================== RUN ALL
    string report = runAll(1);

    //    printAll(report);

    assertAll();

}

TEST_F(Process_Constraint_IMU_ODO, MotionConstant_pqv_b__pqV_b) // F_ixed___e_stimated
{

    // ================================================================================================================ //
    // ==================================== INITIAL CONDITIONS -- USER OPTIONS ======================================== //
    // ================================================================================================================ //
    //
    // ---------- time
    t0                  = 0;
    num_integrations    = 50;

    // ---------- initial pose
    p0                 << 0,0,0;
    q0.coeffs()        << 0,0,0,1;
    v0                 << 0,0,0;

    // ---------- bias
    bias_real          << .001, .002, .003,    -.001, -.002, -.003;
    bias_preint         = -bias_real;

    // ---------- motion params
    a                  = Vector3s( 1,2,3 );
    w                  = Vector3s( 1,2,3 );

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
    string report = runAll(1);

//    printAll(report);

    assertAll();

}

TEST_F(Process_Constraint_IMU_ODO, MotionConstant_pqV_b__pqv_b) // F_ixed___e_stimated
{

    // ================================================================================================================ //
    // ==================================== INITIAL CONDITIONS -- USER OPTIONS ======================================== //
    // ================================================================================================================ //
    //
    // ---------- time
    t0                  = 0;
    num_integrations    = 50;

    // ---------- initial pose
    p0                 << 0,0,0;
    q0.coeffs()        << 0,0,0,1;
    v0                 << 0,0,0;

    // ---------- bias
    bias_real          << .001, .002, .003,    -.001, -.002, -.003;
    bias_preint         = -bias_real;

    // ---------- motion params
    a                  = Vector3s( 1,2,3 );
    w                  = Vector3s( 1,2,3 );

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
    string report = runAll(1);

//    printAll(report);

    assertAll();

}

TEST_F(Process_Constraint_IMU_ODO, MotionRandom_PQV_b__pqv_b) // F_ixed___e_stimated
{

    // ================================================================================================================ //
    // ==================================== INITIAL CONDITIONS -- USER OPTIONS ======================================== //
    // ================================================================================================================ //
    //
    // ---------- time
    t0                  = 0;
    num_integrations    = 50;

    // ---------- initial pose
    p0                 << 0,0,0;
    q0.coeffs()        << 0,0,0,1;
    v0                 << 0,0,0;

    // ---------- bias
    bias_real          << .001, .002, .003,    -.001, -.002, -.003;
    bias_preint         = -bias_real;

    // ---------- motion params
    a                  = Matrix<Scalar, 3, 50>::Random();
    w                  = Matrix<Scalar, 3, 50>::Random();

    // ---------- fix boundaries
    p0_fixed       = true;
    q0_fixed       = true;
    v0_fixed       = true;
    p1_fixed       = false;
    q1_fixed       = false;
    v1_fixed       = false;
    //
    // ===================================== INITIAL CONDITIONS -- USER INPUT ENDS HERE =============================== //
    // ================================================================================================================ //


    // ===================================== RUN ALL
    string report = runAll(1);

    // printAll(report);

    assertAll();

}

TEST_F(Process_Constraint_IMU_ODO, MotionRandom_PqV_b__pqV_b) // F_ixed___e_stimated
{

    // ================================================================================================================ //
    // ==================================== INITIAL CONDITIONS -- USER OPTIONS ======================================== //
    // ================================================================================================================ //
    //
    // ---------- time
    t0                  = 0;
    num_integrations    = 50;

    // ---------- initial pose
    p0                 << 0,0,0;
    q0.coeffs()        << 0,0,0,1;
    v0                 << 0,0,0;

    // ---------- bias
    bias_real          << .001, .002, .003,    -.001, -.002, -.003;
    bias_preint         = -bias_real;

    // ---------- motion params
    a                  = Matrix<Scalar, 3, 50>::Random();
    w                  = Matrix<Scalar, 3, 50>::Random();

    // ---------- fix boundaries
    p0_fixed       = true;
    q0_fixed       = false;
    v0_fixed       = true;
    p1_fixed       = false;
    q1_fixed       = false;
    v1_fixed       = true;
    //
    // ===================================== INITIAL CONDITIONS -- USER INPUT ENDS HERE =============================== //
    // ================================================================================================================ //


    // ===================================== RUN ALL
    string report = runAll(1);

//    printAll(report);

    assertAll();

}

TEST_F(Process_Constraint_IMU_ODO, RecoverTrajectory_MotionRandom_PqV_b__pqV_b) // F_ixed___e_stimated
{

    // ================================================================================================================ //
    // ==================================== INITIAL CONDITIONS -- USER OPTIONS ======================================== //
    // ================================================================================================================ //
    //
    // ---------- time
    t0                  = 0;
    num_integrations    = 25;

    // ---------- initial pose
    p0                 << 0,0,0;
    q0.coeffs()        << 0,0,0,1;
    v0                 << 0,0,0;

    // ---------- bias
    bias_real          << .001, .002, .003,    -.001, -.002, -.003;
    bias_preint         = -bias_real;

    // ---------- motion params
    a  = Matrix<Scalar, 3, 25>::Ones() + 0.1 * Matrix<Scalar, 3, 25>::Random();
    w  = Matrix<Scalar, 3, 25>::Ones() + 0.1 * Matrix<Scalar, 3, 25>::Random();

    // ---------- fix boundaries
    p0_fixed       = true;
    q0_fixed       = false;
    v0_fixed       = true;
    p1_fixed       = false;
    q1_fixed       = false;
    v1_fixed       = true;
    //
    // ===================================== INITIAL CONDITIONS -- USER INPUT ENDS HERE =============================== //
    // ================================================================================================================ //


    // ===================================== RUN ALL
    configureAll();
    integrateAllTrajectories();
    buildProblem();
    string report = solveProblem(1);

    assertAll();

    // Build optimal trajectory
    MatrixXs Trj_x_optim(10,Trj_D_preint_prc.cols());
    Scalar Dt = 0;
    Size i = 0;
    for (auto J_D_b : Buf_Jac_preint_prc)
    {
        VectorXs step       = J_D_b * (bias_0 - bias_preint);
        VectorXs D_opt      = imu::plus(Trj_D_preint_prc.col(i).eval(), step);
        Trj_x_optim.col(i)  = imu::composeOverState(x0_optim, D_opt, Dt);
        Dt += dt;
        i  ++;
    }

    i = 0;
    t = 0;
    MatrixXs Trj_x_optim_prc(10,Trj_D_preint_prc.cols());
    for (int i = 0; i < Trj_x_optim_prc.cols(); i++)
    {
        Trj_x_optim_prc.col(i) = problem->getState(t);
        t += dt;
    }

    // printAll(report);

    WOLF_INFO("------------------------------------");
    WOLF_INFO("Exact x0 \n", x0         .transpose());
    WOLF_INFO("Optim x0 \n", x0_optim   .transpose());
    WOLF_INFO("Optim x  \n", Trj_x_optim.transpose());
    WOLF_INFO("Optim x1 \n", x1_optim   .transpose());
    WOLF_INFO("Exact x1 \n", x1_exact   .transpose());
    WOLF_INFO("------------------------------------");

    WOLF_INFO("------------------------------------");
    WOLF_INFO("Exact x0 \n", x0         .transpose());
    WOLF_INFO("Optim x0 \n", x0_optim   .transpose());
    WOLF_INFO("Optim_prc x  \n", Trj_x_optim_prc.transpose());
    WOLF_INFO("Optim x1 \n", x1_optim   .transpose());
    WOLF_INFO("Exact x1 \n", x1_exact   .transpose());
    WOLF_INFO("------------------------------------");

}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    //    ::testing::GTEST_FLAG(filter) = "Process_Constraint_IMU.*";
    //    ::testing::GTEST_FLAG(filter) = "Process_Constraint_IMU_ODO.*";
        ::testing::GTEST_FLAG(filter) = "Process_Constraint_IMU_ODO.RecoverTrajectory_MotionRandom_PqV_b__pqV_b";

    return RUN_ALL_TESTS();
}



/* Some notes :
 *
 * - Process_Constraint_IMU_ODO.MotionConstant_PQv_b__PQv_b :
 *      this test will not work + jacobian is rank deficient because of estimating both initial
 *      and final velocities. 
 *      IMU data integration is done with correct biases (so this is the case of a calibrated IMU). Before solving the problem, we perturbate the initial bias.
 *      We solve the problem by fixing all states excepted V1 and V2. while creating the constraints, both velocities are corrected using the difference between the actual
 *      bias and the bias used during preintegration. One way to solve this in the solver side would be to make the actual bias match the preintegraion bias so that the
 *      difference is 0 and does not affect the states of the KF. Another possibility is to have both velocities modified without changing the biases. it is likely that this
 *      solution is chosen in this case (bias changes is penalized between 2 KeyFrames, but velocities have no other constraints here.)
 * 
 *  - Bias evaluation with a precision of 1e-4 :
 *      The bias introduced in the data for the preintegration steps is different of the real bias. This is simulating the case of a non calibrated IMU
 *      Because of cross relations between acc and gyro biases (respectively a_b and w_b) it is difficult to expect a better estimation.
 *      A small change in a_b can be cancelled by a small variation in w_b. in other words : there are local minima.
 *      In addition, for Process_Constraint_IMU tests, P and V are tested against 1e-5 precision while 1e-8 is used for Q.
 *      Errors tend to be distributed in different estimated variable when we get into a local minima (to minimize residuals in a least square sense).
 */
