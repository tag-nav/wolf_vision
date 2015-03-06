//std includes
#include <iostream>
#include <memory>
#include <random>

// Eigen includes
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

//Ceres includes
#include "ceres/ceres.h"
#include "glog/logging.h"

class rangeBetweenPoints
{
	private:
		double d_;

	public:
		rangeBetweenPoints(double d):
			d_(d)
		{
			//
		}

		template <typename T>
		bool operator()(const T* const x , const T* const y, T* e) const
		{
			e[0] = T(d_) - ((x[0] - y[0])*(x[0] - y[0]) + (x[1] * y[1])*(x[1] * y[1]));
			return true;
		}
};

//This main is a single iteration of the WOLF. 
//Once at ROS, Calls to WolfVehicle, CeresWolfFunctor and Ceres objects will be executed in a similar order in a function of the ROS wrapper
int main(int argc, char** argv) 
{
    std::cout << " ========= Static Numeric case ===========" << std::endl << std::endl;
    
    //dimension 
    const unsigned int DIM = 19; //just to test, all will be DIM-dimensional
    
    // init
    google::InitGoogleLogging(argv[0]);
    
    // build problem
    double x[2];
    double y[2];
    x[0] = 0;
    x[1] = 0;
    y[0] = 1;
    y[1] = 1;
    ceres::Problem problem;
	problem.AddParameterBlock(x, 2);
	problem.AddParameterBlock(y, 2);
	std::vector<double*> state_blocks({&x[0], &y[0]});

    // cost function
	ceres::CostFunction* range_cost_function1 = new ceres::AutoDiffCostFunction<rangeBetweenPoints, 1, 2, 2>(new rangeBetweenPoints(1.0));
	ceres::CostFunction* range_cost_function2 = new ceres::AutoDiffCostFunction<rangeBetweenPoints, 1, 2, 2>(new rangeBetweenPoints(1.1));

    // Add residual block
    problem.AddResidualBlock(range_cost_function1, nullptr, state_blocks);
    problem.AddResidualBlock(range_cost_function2, nullptr, state_blocks);

    // Solve
    ceres::Solver::Options solver_options;
	ceres::Solver::Summary summary;
	ceres::Solve(solver_options, &problem, &summary);

	//display results
	std::cout << "Ceres Report:" << std::endl;
	std::cout << summary.FullReport() << "\n";

    // Compute covariance
	ceres::Covariance::Options covariance_options;
	ceres::Covariance covariance(covariance_options);

    std::vector<std::pair<const double*, const double*> > covariance_blocks;
    covariance_blocks.push_back(std::make_pair(x, x));
    covariance_blocks.push_back(std::make_pair(y, y));
    covariance_blocks.push_back(std::make_pair(x, y));

    CHECK(covariance.Compute(covariance_blocks, &problem));

    double covariance_xx[3 * 3];
    double covariance_yy[2 * 2];
    double covariance_xy[3 * 2];
    covariance.GetCovarianceBlock(x, x, covariance_xx);
    covariance.GetCovarianceBlock(y, y, covariance_yy);
    covariance.GetCovarianceBlock(x, y, covariance_xy);

    std::cout << "Covariances:" << std::endl;
    std::cout << "Marginal x:" << covariance_xx << std::endl;
    std::cout << "Marginal y:" << covariance_yy << std::endl;
    std::cout << "Cross xy:" << covariance_xy << std::endl;
       
    std::cout << " ========= END ===========" << std::endl << std::endl;
       
    //exit
    return 0;
}

