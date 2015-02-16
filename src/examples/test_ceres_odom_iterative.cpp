// Testing a full wolf tree avoiding template classes for NodeLinked derived classes

//std includes
#include <cstdlib>
#include <stdlib.h> //getenv
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <list>
#include <random>
#include <cmath>
// #include <memory>
// #include <typeinfo>

//ceres
#include "ceres/ceres.h"

//Wolf includes
#include "wolf.h"
#include "node_terminus.h"
#include "node_linked.h"

//forward declarations
class TrajectoryBaseX;
class FrameBaseX;
class CaptureBaseX;
class FeatureBaseX;
class CorrespondenceBaseX;

//class TrajectoryBaseX
class TrajectoryBaseX : public NodeLinked<NodeTerminus,FrameBaseX>
{
    protected:
        unsigned int length_; //just something to play
        
    public:
        TrajectoryBaseX(const unsigned int _len) :
            NodeLinked(TOP, "TRAJECTORY"),
            length_(_len)
        {
            //
        };
        
        virtual ~TrajectoryBaseX()
        {
            
        };
};

//class FrameBaseX
class FrameBaseX : public NodeLinked<TrajectoryBaseX,CaptureBaseX>
{
    protected:
        double time_stamp_; //frame ts
        
    public:
        FrameBaseX(double _ts) :
            NodeLinked(MID, "FRAME"),
            time_stamp_(_ts)
        {
            //
        };
        
        virtual ~FrameBaseX()
        {
            
        };
};

//class CaptureBaseX
class CaptureBaseX : public NodeLinked<FrameBaseX,FeatureBaseX>
{
    protected:
        double time_stamp_; //capture ts
        
    public:
        CaptureBaseX(double _ts) :
            NodeLinked(MID, "CAPTURE"),
            time_stamp_(_ts)
        {
            //
        };
        
        virtual ~CaptureBaseX()
        {
            
        };
};

//class FeatureBaseX
class FeatureBaseX : public NodeLinked<CaptureBaseX,CorrespondenceBaseX>
{
    protected:
        
    public:
        FeatureBaseX() :
            NodeLinked(MID, "FEATURE")
        {
            //
        };
        
        virtual ~FeatureBaseX()
        {
            //
        };
};

//class CorrespondenceBaseX
class CorrespondenceBaseX : public NodeLinked<FeatureBaseX,NodeTerminus>
{
    protected:
        unsigned int nblocks_; //number of state blocks in which the correspondence depends on.
        //std::vector<unsigned int> block_indexes_; //state vector indexes indicating start of each state block. This vector has nblocks_ size. 
        std::vector<unsigned int> block_sizes_; //sizes of each state block. This vector has nblocks_ size. 
        ceres::CostFunction* cost_function_ptr_;
        ceres::ResidualBlockId ceres_residual_block_id_;
        
    public:
        //CorrespondenceBaseX(const unsigned int _nb, const std::vector<unsigned int> & _bindexes, const std::vector<unsigned int> & _bsizes) :
        CorrespondenceBaseX(const unsigned int _nb, const std::vector<unsigned int> & _bsizes) :
            NodeLinked(BOTTOM, "CORRESPONDENCE"),
            nblocks_(_nb),
            //block_indexes_(_bindexes),
            block_sizes_(_bsizes)
        {
            assert(block_sizes_.size() == nblocks_);
        };
        
        virtual ~CorrespondenceBaseX()
        {
            //
        };
        
//         ceres::CostFunction * getCostFunctionPtr()
//         {
//             return cost_function_ptr_;
//         };
        
        virtual void addToProblem(ceres::Problem & _problem) = 0;
        
        virtual void removeFromProblem(ceres::Problem & _problem)
        {
            _problem.RemoveResidualBlock(ceres_residual_block_id_);
        }
        	
        virtual void display() const
        {
                unsigned int ii; 
                std::cout << "number of state blocks: " << nblocks_ << std::endl;
                //std::cout << "state block indexes: ";
                //for (ii=0; ii<block_indexes_.size(); ii++) std::cout << block_indexes_.at(ii) << " ";
                std::cout << std::endl;
                std::cout << "state block sizes: ";
                for (ii=0; ii<block_sizes_.size(); ii++) std::cout << block_sizes_.at(ii) << " ";
                std::cout << std::endl;
                std::cout << "ceres residual block id: " << ceres_residual_block_id_ << std::endl;
        };
};

class Odom2DFunctor
{
    protected:
        Eigen::Vector2s odom_inc_; //incremental odometry measurements (range, theta). Could be a map to data hold by capture or feature
        const double odom_stdev_ = 0.01; //model parameters
    
    public:
        //constructor
        Odom2DFunctor(const Eigen::Vector2s & _odom):
            odom_inc_(_odom) 
        {
            //
        };
        
        //destructor
        virtual ~Odom2DFunctor()
        {
            //
        };
        
        //cost function
        template <typename T>
        bool operator()(const T* const _x0, const T* const _x1, T* _residual) const
        {
            T dr2, dth;
            
            //expected range and theta increments, given the state points
            dr2 = (_x0[0]-_x1[0])*(_x0[0]-_x1[0]) + (_x0[1]-_x1[1])*(_x0[1]-_x1[1]); //square of the range
            dth = _x1[2] - _x0[2]; //
            
            //residuals in range and theta components 
            _residual[0] = (T(dr2) - T(odom_inc_(0)*odom_inc_(0))) / T(odom_stdev_);
            _residual[1] = (T(dth) - T(odom_inc_(1))) / T(odom_stdev_);
            
            //return 
            return true;
        };
};

//Specialized correspondence class for 2D odometry
class CorrespondenceOdom2D : public CorrespondenceBaseX
{
    protected:
        Eigen::Map<Eigen::Vector3s> pose_previous_;
        Eigen::Map<Eigen::Vector3s> pose_current_; 
        Eigen::Map<const Eigen::Vector2s> odom_inc_; 
        
    public:
        CorrespondenceOdom2D(WolfScalar * _st_prev, WolfScalar * _st_curr, const Eigen::Vector2s & _odom) :
            CorrespondenceBaseX(2,{3,3}),
            pose_previous_(_st_prev),//, block_sizes_.at(0)), //size 3 is already defined at declaration
            pose_current_(_st_curr),//, block_sizes_.at(1)), //size 3 is already defined at declaration
            odom_inc_(_odom.data())
        {
            cost_function_ptr_ = new ceres::AutoDiffCostFunction<Odom2DFunctor,2,3,3>(new Odom2DFunctor(_odom));
        };
        
        virtual ~CorrespondenceOdom2D()
        {
            //delete cost_function_ptr_;
        };
                
//         double * getPosePreviousPtr()
//         {
//             return pose_previous_.data();
//         };
//         
//         double * getPoseCurrentPtr()
//         {
//             return pose_current_.data();
//         };        
        
        virtual void addToProblem(ceres::Problem & _problem)
        {
            ceres_residual_block_id_ = _problem.AddResidualBlock(cost_function_ptr_,nullptr, pose_previous_.data(), pose_current_.data());
        }

        virtual void display() const
        {
            std::cout << "---- Odom Correspondence ----" << std::endl;
            CorrespondenceBaseX::display();
            std::cout << "pose_previous_: " << pose_previous_.transpose() << std::endl;
            std::cout << "pose_current_: " << pose_current_.transpose() << std::endl;
            std::cout << "odom_inc_: " << odom_inc_.transpose() << std::endl;
        };      
};

class GPSFixFunctor
{
    protected:
        Eigen::Vector3s gps_fix_; //GPS fix XYZ. Could be a map to data hold by capture or feature
        const double gps_stdev_ = 1; //model parameters
    
    public:
        //constructor
        GPSFixFunctor(const Eigen::Vector3s & _gps_fix):
            gps_fix_(_gps_fix)
        {
            //
        };
        
        //destructor
        virtual ~GPSFixFunctor()
        {
            //
        };
        
        //cost function
        template <typename T>
        bool operator()(const T* const _x0, T* _residual) const
        {
            T dist_x = T( gps_fix_(0) ) - _x0[0];
            T dist_y = T( gps_fix_(1) ) - _x0[1];
            _residual[0] = dist_x / T(gps_stdev_);
            _residual[1] = dist_y / T(gps_stdev_);
            _residual[2] = T(0.); //T( gps_fix_(2) ) - _x0[2]; //in this case, third component of the state is heading, not Z, so it is ignored
            return true;
        };
};

//Specialized correspondence class for GPS Fix data
class CorrespondenceGPSFix : public CorrespondenceBaseX
{
    protected:
        Eigen::Map<Eigen::Vector3s> state_position_;
        Eigen::Map<const Eigen::Vector3s> gps_fix_;

    public:
        CorrespondenceGPSFix(WolfScalar * _st, const Eigen::Vector3s & _gps_fix) :
            CorrespondenceBaseX(1,{3}), 
            state_position_(_st),// block_sizes_.at(0)), //size 3 is already defined at declaration
            gps_fix_(_gps_fix.data())
        {
            cost_function_ptr_ = new ceres::AutoDiffCostFunction<GPSFixFunctor,3,3>(new GPSFixFunctor(_gps_fix));
        };
        
        virtual ~CorrespondenceGPSFix()
        {
            //delete cost_function_ptr_;
        };
        
//         double * getLocation()
//         {
//             return location_.data();
//         }
        
        virtual void addToProblem(ceres::Problem & _problem)
        {
            ceres_residual_block_id_ = _problem.AddResidualBlock(cost_function_ptr_,nullptr,state_position_.data()); 
        }
                
        virtual void display() const
        {
            std::cout << "---- GPS Fix Correspondence ----" << std::endl;
            CorrespondenceBaseX::display();
            std::cout << "state_position_: " << state_position_.transpose() << std::endl;
            std::cout << "gps_fix_: " << gps_fix_.transpose() << std::endl;            
        };        
};


int main(int argc, char** argv) 
{    
    //Welcome message
    std::cout << std::endl << " ========= WOLF-CERES test. Simple Odometry + GPS fix problem (with non-template classes) ===========" << std::endl << std::endl;

    //user input
    if (argc!=3)
    {
        std::cout << "Please call me with: [./test_ceres_odom_iterative NI NW], where:" << std::endl;
        std::cout << "       - NI is the number of iterations" << std::endl;
        std::cout << "       - NW is the size of the window" << std::endl;
        std::cout << "EXIT due to bad user input" << std::endl << std::endl;
        return -1;
    }
    unsigned int n_execution = (unsigned int) atoi(argv[1]); //number of iterations of the whole execution
    unsigned int n_window = (unsigned int) atoi(argv[2]); //size of the window.
    
    //init google log
    google::InitGoogleLogging(argv[0]);

    //variables    
    unsigned int ii, jj, jj_previous; //iterators
    Eigen::VectorXs odom_inc_true;//invented motion
    Eigen::Vector3s pose_true; //current true pose
    Eigen::VectorXs ground_truth; //accumulated true poses
    Eigen::Vector3s pose_predicted; // current predicted pose
    Eigen::VectorXs state; //running window winth solver result
    Eigen::VectorXs state_prior; //state prior, just before solving the problem
    Eigen::Vector2s odom_reading; //current odometry reading
    Eigen::Vector3s gps_fix_reading; //current GPS fix reading
    Eigen::VectorXs gps_log; //log of all gps readings
    Eigen::VectorXs results_log; //log of optimized poses
    CorrespondenceOdom2D *odom_corresp; //pointer to odometry correspondence
    CorrespondenceGPSFix *gps_fix_corresp; //pointer to GPS fix correspondence
    ceres::Problem problem; //ceres problem 
    ceres::Solver::Options options; //ceres solver options
    ceres::Solver::Summary summary; //ceres solver summary
    std::ofstream log_file;  //output file

    //resize vectors according user input number of iterations
    odom_inc_true.resize(n_execution*2); //2 odometry components per iteration
    ground_truth.resize(n_execution*3);// 3 components per iteration
    gps_log.resize(n_execution*3); //3 components per iteration
    results_log.resize(n_execution*3); //3 components per iteration
    state.resize(n_window*3); //3 components per window element
    state_prior.resize(n_execution*3); //3 components per window element
    
    //init true odom and true pose
    for (ii = 0; ii<n_execution; ii++)
    {
        if ( ii < (unsigned int)floor(n_execution/2) )
            odom_inc_true.middleRows(ii*2,2) << fabs(cos(ii/10.)) , fabs(sin(ii/2000.)); //invented motion increments. 
        else
            odom_inc_true.middleRows(ii*2,2) << fabs(cos(ii/10.)) , -fabs(sin((ii-floor(n_execution/2))/2000.)); //invented motion increments. 
    }
    pose_true << 0,0,0;
    pose_predicted << 0,0,0;
    ground_truth.middleRows(0,3) << pose_true; //init point pushed to ground truth
    state.middleRows(0,3) << 0,0,0; //init state at origin
    
    //init random generators
    std::default_random_engine generator;
    std::normal_distribution<WolfScalar> distribution_odom(0.001,0.01); //odometry noise
    std::normal_distribution<WolfScalar> distribution_gps(0.0,1); //GPS noise
    
    //test loop
    for (ii = 1; ii<n_execution; ii++) //ii over iterations, jj over the window
    {
        //set jj index (over the window)
        jj = ii%n_window;
        jj_previous = (ii-1)%n_window;
        
        //inventing a simple motion
        pose_true(0) = pose_true(0) + odom_inc_true(ii*2) * cos(pose_true(2)+odom_inc_true(ii*2+1)); 
        pose_true(1) = pose_true(1) + odom_inc_true(ii*2) * sin(pose_true(2)+odom_inc_true(ii*2+1)); 
        pose_true(2) = pose_true(2) + odom_inc_true(ii*2+1);
        ground_truth.middleRows(ii*3,3) << pose_true;
        //std::cout << "pose_true(" << ii << ") = " << pose_true.transpose() << std::endl;
        
        //inventing sensor readings for odometry and GPS
        odom_reading << odom_inc_true(ii*2)+distribution_odom(generator), odom_inc_true(ii*2+1)+distribution_odom(generator); //true range and theta with noise
        gps_fix_reading << pose_true(0) + distribution_gps(generator), pose_true(1) + distribution_gps(generator), 0. + distribution_gps(generator);
        gps_log.middleRows(ii*3,3) << gps_fix_reading;//log the reading
        
        //setting initial guess from the last optimized pose, using noisy odometry
        pose_predicted(0) = state(jj_previous*3) + odom_reading(0) * cos(state(jj_previous*3+2)+odom_reading(1)); 
        pose_predicted(1) = state(jj_previous*3+1) + odom_reading(0) * sin(state(jj_previous*3+2)+odom_reading(1)); 
        pose_predicted(2) = state(jj_previous*3+2) + odom_reading(1);
        
        //window management
        state.middleRows(jj*3,3) << pose_predicted;
        
        //creating odom correspondence. Adding it to the problem 
        odom_corresp = new CorrespondenceOdom2D(state.data()+jj_previous*3, state.data()+jj*3, odom_reading);
        odom_corresp->addToProblem(problem);
        //odom_corresp->display();
        delete odom_corresp;
        
        //creating gps correspondence and adding it to the problem 
        gps_fix_corresp = new CorrespondenceGPSFix(state.data()+ii*3, gps_fix_reading);
        gps_fix_corresp->addToProblem(problem);
        //gps_fix_corresp->display();
        delete gps_fix_corresp;
        
        //set options and solve (sliding window)
//         options.minimizer_progress_to_stdout = true;
//         ceres::Solve(options, &problem, &summary);
    }
    
    //display initial guess
    //std::cout << "INITIAL GUESS IS: " << state.transpose() << std::endl;
    
    //set options and solve (batch mode)
    //options.minimizer_progress_to_stdout = true;
    state_prior = state;
    options.minimizer_type = ceres::LINE_SEARCH;//ceres::TRUST_REGION;
    options.max_line_search_step_contraction = 1e-3;
    ceres::Solve(options, &problem, &summary);
    
    //display/log results, by setting cout flags properly
    std::string filename( getenv("HOME") );
    filename += "/Desktop/log_data.txt";
    std::cout << std::endl << " Result to file " << filename << std::endl;
    log_file.open(filename, std::ofstream::out); //open log file
    for (unsigned int ii = 0; ii<n_execution; ii++) 
        log_file << state.middleRows(ii*3,3).transpose() << " " << ground_truth.middleRows(ii*3,3).transpose() << " " << (state.middleRows(ii*3,3)-ground_truth.middleRows(ii*3,3)).transpose() << " " << gps_log.middleRows(ii*3,3).transpose() << " " <<  state_prior.middleRows(ii*3,3).transpose() <<std::endl;        
    log_file.close(); //close log file
  
    //free memory (not necessary since ceres::problem holds their ownership)
//     delete odom_corresp;
//     delete gps_fix_corresp;
    
    //End message
    std::cout << " =========================== END ===============================" << std::endl << std::endl;
       
    //exit
    return 0;
}
