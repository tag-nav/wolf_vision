// Testing creating wolf tree from imported .graph file

//std includes
#include <iostream>
#include <memory>
#include <random>
#include <cmath>
#include <queue>

//Wolf includes
#include "wolf_manager.h"
#include "capture_void.h"
#include "ceres_wrapper/ceres_manager.h"

int main(int argc, char** argv) 
{
    //Welcome message
    std::cout << std::endl << " ========= WOLF IMPORTED .graph TEST ===========" << std::endl << std::endl;

    if (argc != 2 )
    {
        std::cout << "Please call me with: [./test_wolf_imported_graph FILE_PATH], where:" << std::endl;
        std::cout << "    FILE_PATH is the .graph file path" << std::endl;
        std::cout << "EXIT due to bad user input" << std::endl << std::endl;
        return -1;
    }

    std::string file_path_ = argv[1];

    WolfProblem* wolf_problem_ = new WolfProblem();

    std::map<unsigned int, FrameBase*> index_2_frame_ptr_;
    std::ifstream offLineFile_;

    unsigned int vertex_index;
    Eigen::Vector3s vertex_pose;
    FrameBase* vertex_frame_prt;

    unsigned int edge_from, edge_to;
    Eigen::Vector3s edge_vector;
    Eigen::Matrix3s edge_covariance;
    CaptureVoid* capture_ptr;
    FeatureBase* feature_ptr;
    FrameBase* frame_to_ptr;
    FrameBase* frame_from_ptr;
    ConstraintOdom2D* constraint_ptr;

    // load graph from .txt
    offLineFile_.open(file_path_.c_str(), std::ifstream::in);
    if (offLineFile_.is_open())
    {
        std::string buffer;
        // Line by line
        while (std::getline(offLineFile_, buffer))
        {
            std::cout << "new line:" << buffer << std::endl;
            std::string bNum;
            unsigned int i = 0;

            // VERTEX
            if (buffer.at(0) == 'V')
            {
                //skip rest of VERTEX word
                while (buffer.at(i) != ' ') i++;
                //skip white spaces
                while (buffer.at(i) == ' ') i++;

                //vertex index
                while (buffer.at(i) != ' ')
                    bNum.push_back(buffer.at(i++));
                vertex_index = atoi(bNum.c_str());
                bNum.clear();
                //skip white spaces
                while (buffer.at(i) == ' ') i++;

                // vertex pose
                // x
                while (buffer.at(i) != ' ')
                    bNum.push_back(buffer.at(i++));
                vertex_pose(0) = atof(bNum.c_str());
                bNum.clear();
                //skip white spaces
                while (buffer.at(i) == ' ') i++;
                // y
                while (buffer.at(i) != ' ')
                    bNum.push_back(buffer.at(i++));
                vertex_pose(1) = atof(bNum.c_str());
                bNum.clear();
                //skip white spaces
                while (buffer.at(i) == ' ') i++;
                // theta
                while (i < buffer.size() && buffer.at(i) != ' ')
                    bNum.push_back(buffer.at(i++));
                vertex_pose(2) = atof(bNum.c_str());
                bNum.clear();

                // add frame to problem
                vertex_frame_prt = new FrameBase(TimeStamp(0), new StateBlock(vertex_pose.head(2)), new StateBlock(vertex_pose.tail(1)));
                wolf_problem_->getTrajectoryPtr()->addFrame(vertex_frame_prt);
                // store
                index_2_frame_ptr_[vertex_index] = vertex_frame_prt;
                std::cout << "Added vertex! index :" << vertex_index << " pose: " << vertex_pose.transpose() << std::endl;
            }
            // EDGE
            else if (buffer.at(0) == 'E')
            {
                //skip rest of EDGE word
                while (buffer.at(i) != ' ') i++;
                //skip white spaces
                while (buffer.at(i) == ' ') i++;

                //from
                while (buffer.at(i) != ' ')
                    bNum.push_back(buffer.at(i++));
                edge_from = atoi(bNum.c_str());
                printf("edge from %i\n", edge_from);
                assert(index_2_frame_ptr_.find(edge_from) != index_2_frame_ptr_.end() && "edge from vertex not added!");
                frame_from_ptr = index_2_frame_ptr_[edge_from];
                bNum.clear();
                //skip white spaces
                while (buffer.at(i) == ' ') i++;

                //to index
                while (buffer.at(i) != ' ')
                    bNum.push_back(buffer.at(i++));
                edge_to = atoi(bNum.c_str());
                printf("edge to %i\n", edge_to);
                assert(index_2_frame_ptr_.find(edge_to) != index_2_frame_ptr_.end() && "edge to vertex not added!");
                frame_to_ptr = index_2_frame_ptr_[edge_to];
                bNum.clear();
                //skip white spaces
                while (buffer.at(i) == ' ') i++;

                // edge vector
                // x
                while (buffer.at(i) != ' ')
                    bNum.push_back(buffer.at(i++));
                edge_vector(0) = atof(bNum.c_str());
                bNum.clear();
                //skip white spaces
                while (buffer.at(i) == ' ') i++;
                // y
                while (buffer.at(i) != ' ')
                    bNum.push_back(buffer.at(i++));
                edge_vector(1) = atof(bNum.c_str());
                bNum.clear();
                //skip white spaces
                while (buffer.at(i) == ' ') i++;
                // theta
                while (buffer.at(i) != ' ')
                    bNum.push_back(buffer.at(i++));
                edge_vector(2) = atof(bNum.c_str());
                bNum.clear();

                // edge covariance
                // xx
                while (buffer.at(i) != ' ')
                    bNum.push_back(buffer.at(i++));
                edge_covariance(0,0) = atof(bNum.c_str());
                bNum.clear();
                //skip white spaces
                while (buffer.at(i) == ' ') i++;
                // xy
                while (buffer.at(i) != ' ')
                    bNum.push_back(buffer.at(i++));
                edge_covariance(0,1) = atof(bNum.c_str());
                edge_covariance(1,0) = atof(bNum.c_str());
                bNum.clear();
                //skip white spaces
                while (buffer.at(i) == ' ') i++;
                // yy
                while (buffer.at(i) != ' ')
                    bNum.push_back(buffer.at(i++));
                edge_covariance(1,1) = atof(bNum.c_str());
                bNum.clear();
                //skip white spaces
                while (buffer.at(i) == ' ') i++;
                // thetatheta
                while (buffer.at(i) != ' ')
                    bNum.push_back(buffer.at(i++));
                edge_covariance(2,2) = atof(bNum.c_str());
                bNum.clear();
                //skip white spaces
                while (buffer.at(i) == ' ') i++;
                // xtheta
                while (buffer.at(i) != ' ')
                    bNum.push_back(buffer.at(i++));
                edge_covariance(0,2) = atof(bNum.c_str());
                edge_covariance(2,0) = atof(bNum.c_str());
                bNum.clear();
                //skip white spaces
                while (buffer.at(i) == ' ') i++;
                // ytheta
                while (buffer.at(i) != ' ')
                    bNum.push_back(buffer.at(i++));
                edge_covariance(1,2) = atof(bNum.c_str());
                edge_covariance(2,1) = atof(bNum.c_str());
                bNum.clear();

                // add capture, feature and constraint to problem
                feature_ptr = new FeatureBase(edge_vector, edge_covariance.inverse());
                printf("feature_ptr!\n");
                capture_ptr = new CaptureVoid(TimeStamp(0), nullptr);
                printf("capture_ptr!\n");
                frame_from_ptr->addCapture(capture_ptr);
                printf("addCapture!\n");
                capture_ptr->addFeature(feature_ptr);
                printf("addFeature!\n");
                constraint_ptr = new ConstraintOdom2D(feature_ptr, frame_to_ptr);
                printf("constraint_ptr!\n");
                feature_ptr->addConstraintFrom(constraint_ptr);
                printf("addConstraintFrom!\n");
            }
            else
                assert("unknown line");
        }
        printf("\nGraph loaded!\n");
    }
    else
        printf("\nError opening file\n");

    // SOLVING
    // Ceres wrapper
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    //    ceres_options.minimizer_progress_to_stdout = false;
    //    ceres_options.line_search_direction_type = ceres::LBFGS;
    //    ceres_options.max_num_iterations = 100;
    ceres::Problem::Options problem_options;
    problem_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    problem_options.loss_function_ownership = ceres::TAKE_OWNERSHIP;//ceres::DO_NOT_TAKE_OWNERSHIP;
    problem_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    CeresManager* ceres_manager = new CeresManager(wolf_problem_, problem_options);


    ceres_manager->update();
    ceres_manager->computeCovariances(ALL);
    ceres::Solver::Summary summary = ceres_manager->solve(ceres_options);
    std::cout << summary.FullReport() << std::endl;

    delete wolf_problem_; //not necessary to delete anything more, wolf will do it!
    delete ceres_manager;
    //End message
    std::cout << " =========================== END ===============================" << std::endl << std::endl;
       
    //exit
    return 0;
}
