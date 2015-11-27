// Testing creating wolf tree from imported .graph file

//C includes for sleep, time and main args
#include "unistd.h"

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

    if (argc != 3 || atoi(argv[2]) < 0 )
    {
        std::cout << "Please call me with: [./test_wolf_imported_graph FILE_PATH MAX_VERTICES], where:" << std::endl;
        std::cout << "    FILE_PATH is the .graph file path" << std::endl;
        std::cout << "    MAX_VERTICES > 0 max edges to be loaded" << std::endl;
        std::cout << "EXIT due to bad user input" << std::endl << std::endl;
        return -1;
    }

    // auxiliar variables
    std::string file_path_ = argv[1];
    unsigned int MAX_VERTEX = atoi(argv[2]);
    std::ifstream offLineFile_;
    clock_t t1, t2;

    // loading variables
    std::map<unsigned int, FrameBase*> index_2_frame_ptr_all;
    std::map<unsigned int, FrameBase*> index_2_frame_ptr_prunned;

    // Wolf problem
    WolfProblem* wolf_problem_all = new WolfProblem();
    WolfProblem* wolf_problem_prunned = new WolfProblem();

    // Ceres wrapper
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    ceres_options.max_num_iterations = 1e4;
    ceres::Problem::Options problem_options;
    problem_options.cost_function_ownership = ceres::TAKE_OWNERSHIP;
    problem_options.loss_function_ownership = ceres::TAKE_OWNERSHIP;
    problem_options.local_parameterization_ownership = ceres::TAKE_OWNERSHIP;
    CeresManager* ceres_manager_all = new CeresManager(wolf_problem_all, problem_options);
    CeresManager* ceres_manager_prunned = new CeresManager(wolf_problem_prunned, problem_options);

    // load graph from .txt
    offLineFile_.open(file_path_.c_str(), std::ifstream::in);
    if (offLineFile_.is_open())
    {
        std::string buffer;
        unsigned int j = 0;
        // Line by line
        while (std::getline(offLineFile_, buffer))
        {
            //std::cout << "new line:" << buffer << std::endl;
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
                unsigned int vertex_index = atoi(bNum.c_str());
                bNum.clear();

                if (vertex_index <= MAX_VERTEX)
                {
                    //skip white spaces
                    while (buffer.at(i) == ' ') i++;

                    // vertex pose
                    Eigen::Vector3s vertex_pose;
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
                    FrameBase* vertex_frame_ptr_all = new FrameBase(TimeStamp(0), new StateBlock(vertex_pose.head(2)), new StateBlock(vertex_pose.tail(1)));
                    FrameBase* vertex_frame_ptr_prunned = new FrameBase(TimeStamp(0), new StateBlock(vertex_pose.head(2)), new StateBlock(vertex_pose.tail(1)));
                    wolf_problem_all->getTrajectoryPtr()->addFrame(vertex_frame_ptr_all);
                    wolf_problem_prunned->getTrajectoryPtr()->addFrame(vertex_frame_ptr_prunned);
                    // store
                    index_2_frame_ptr_all[vertex_index] = vertex_frame_ptr_all;
                    index_2_frame_ptr_prunned[vertex_index] = vertex_frame_ptr_prunned;
                    std::cout << "Added vertex! index: " << vertex_index << " nodeId: " << vertex_frame_ptr_prunned->nodeId() << std::endl << "pose: " << vertex_pose.transpose() << std::endl;
                }
            }
            // EDGE
            else if (buffer.at(0) == 'E')
            {
                j++;
                //skip rest of EDGE word
                while (buffer.at(i) != ' ') i++;
                //skip white spaces
                while (buffer.at(i) == ' ') i++;

                //from
                while (buffer.at(i) != ' ')
                    bNum.push_back(buffer.at(i++));
                unsigned int edge_from = atoi(bNum.c_str());
                bNum.clear();
                //skip white spaces
                while (buffer.at(i) == ' ') i++;

                //to index
                while (buffer.at(i) != ' ')
                    bNum.push_back(buffer.at(i++));
                unsigned int edge_to = atoi(bNum.c_str());
                bNum.clear();

                if (edge_to <= MAX_VERTEX && edge_from <= MAX_VERTEX )
                {
                    //skip white spaces
                    while (buffer.at(i) == ' ') i++;

                    // edge vector
                    Eigen::Vector3s edge_vector;
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
                    //skip white spaces
                    while (buffer.at(i) == ' ') i++;

                    // edge covariance
                    Eigen::Matrix3s edge_information;
                    // xx
                    while (buffer.at(i) != ' ')
                        bNum.push_back(buffer.at(i++));
                    edge_information(0,0) = atof(bNum.c_str());
                    bNum.clear();
                    //skip white spaces
                    while (buffer.at(i) == ' ') i++;
                    // xy
                    while (buffer.at(i) != ' ')
                        bNum.push_back(buffer.at(i++));
                    edge_information(0,1) = atof(bNum.c_str());
                    edge_information(1,0) = atof(bNum.c_str());
                    bNum.clear();
                    //skip white spaces
                    while (buffer.at(i) == ' ') i++;
                    // yy
                    while (buffer.at(i) != ' ')
                        bNum.push_back(buffer.at(i++));
                    edge_information(1,1) = atof(bNum.c_str());
                    bNum.clear();
                    //skip white spaces
                    while (buffer.at(i) == ' ') i++;
                    // thetatheta
                    while (buffer.at(i) != ' ')
                        bNum.push_back(buffer.at(i++));
                    edge_information(2,2) = atof(bNum.c_str());
                    bNum.clear();
                    //skip white spaces
                    while (buffer.at(i) == ' ') i++;
                    // xtheta
                    while (buffer.at(i) != ' ')
                        bNum.push_back(buffer.at(i++));
                    edge_information(0,2) = atof(bNum.c_str());
                    edge_information(2,0) = atof(bNum.c_str());
                    bNum.clear();
                    //skip white spaces
                    while (buffer.at(i) == ' ') i++;
                    // ytheta
                    while (i < buffer.size() && buffer.at(i) != ' ')
                        bNum.push_back(buffer.at(i++));
                    edge_information(1,2) = atof(bNum.c_str());
                    edge_information(2,1) = atof(bNum.c_str());
                    bNum.clear();

                    // add capture, feature and constraint to problem
                    FeatureBase* feature_ptr_all = new FeatureBase(edge_vector, edge_information.inverse());
                    FeatureBase* feature_ptr_prunned = new FeatureBase(edge_vector, edge_information.inverse());
                    CaptureVoid* capture_ptr_all = new CaptureVoid(TimeStamp(0), nullptr);
                    CaptureVoid* capture_ptr_prunned = new CaptureVoid(TimeStamp(0), nullptr);
                    assert(index_2_frame_ptr_all.find(edge_from) != index_2_frame_ptr_all.end() && "edge from vertex not added!");
                    assert(index_2_frame_ptr_prunned.find(edge_from) != index_2_frame_ptr_prunned.end() && "edge from vertex not added!");
                    FrameBase* frame_from_ptr_all = index_2_frame_ptr_all[edge_from];
                    FrameBase* frame_from_ptr_prunned = index_2_frame_ptr_prunned[edge_from];
                    frame_from_ptr_all->addCapture(capture_ptr_all);
                    frame_from_ptr_prunned->addCapture(capture_ptr_prunned);
                    capture_ptr_all->addFeature(feature_ptr_all);
                    capture_ptr_prunned->addFeature(feature_ptr_prunned);
                    assert(index_2_frame_ptr_all.find(edge_to) != index_2_frame_ptr_all.end() && "edge to vertex not added!");
                    assert(index_2_frame_ptr_prunned.find(edge_to) != index_2_frame_ptr_prunned.end() && "edge to vertex not added!");
                    FrameBase* frame_to_ptr_all = index_2_frame_ptr_all[edge_to];
                    FrameBase* frame_to_ptr_prunned = index_2_frame_ptr_prunned[edge_to];
                    ConstraintOdom2D* constraint_ptr_all = new ConstraintOdom2D(feature_ptr_all, frame_to_ptr_all);
                    ConstraintOdom2D* constraint_ptr_prunned = new ConstraintOdom2D(feature_ptr_prunned, frame_to_ptr_prunned);
                    feature_ptr_all->addConstraintFrom(constraint_ptr_all);
                    feature_ptr_prunned->addConstraintFrom(constraint_ptr_prunned);
                    std::cout << "Added edge! " << constraint_ptr_prunned->nodeId() << " from vertex " << constraint_ptr_prunned->getCapturePtr()->getFramePtr()->nodeId() << " to " << constraint_ptr_prunned->getFrameToPtr()->nodeId() << std::endl;
                    //std::cout << "vector " << constraint_ptr_prunned->getMeasurement().transpose() << std::endl;
                    //std::cout << "information " << std::endl << edge_information << std::endl;
                    //std::cout << "covariance " << std::endl << constraint_ptr_prunned->getMeasurementCovariance() << std::endl;
                }
            }
            else
                assert("unknown line");
        }
        printf("\nGraph loaded!\n");
    }
    else
        printf("\nError opening file\n");

    // PRIOR
    FrameBase* first_frame_all = wolf_problem_all->getTrajectoryPtr()->getFrameListPtr()->front();
    FrameBase* first_frame_prunned = wolf_problem_prunned->getTrajectoryPtr()->getFrameListPtr()->front();
    CaptureFix* initial_covariance_all = new CaptureFix(TimeStamp(0), first_frame_all->getState(), Eigen::Matrix3s::Identity() * 0.01);
    CaptureFix* initial_covariance_prunned = new CaptureFix(TimeStamp(0), first_frame_prunned->getState(), Eigen::Matrix3s::Identity() * 0.01);
    first_frame_all->addCapture(initial_covariance_all);
    first_frame_prunned->addCapture(initial_covariance_prunned);
    initial_covariance_all->processCapture();
    initial_covariance_prunned->processCapture();

    // BUILD SOLVER PROBLEM
    std::cout << "updating ceres..." << std::endl;
    ceres_manager_all->update();
    ceres_manager_prunned->update();
    std::cout << "updated!" << std::endl;

    // PRUNNING
    std::cout << "computing covariances..." << std::endl;
    ceres_manager_prunned->computeCovariances(ALL);
    std::cout << "computed!" << std::endl;
    ConstraintBaseList constraints;
    wolf_problem_prunned->getTrajectoryPtr()->getConstraintList(constraints);

    Eigen::MatrixXs Sigma_ii(3,3), Sigma_ij(3,3), Sigma_jj(3,3), Sigma_z(3,3), Ji(3,3), Jj(3,3);
    WolfScalar xi, yi, thi, si, ci, xj, yj;
    for (auto c_it=constraints.begin(); c_it!=constraints.end(); c_it++)
    {
        if ((*c_it)->getCategory() != CTR_FRAME) continue;

        // ii
        wolf_problem_prunned->getCovarianceBlock((*c_it)->getCapturePtr()->getFramePtr()->getPPtr(), (*c_it)->getCapturePtr()->getFramePtr()->getPPtr(), Sigma_ii, 0, 0);
        wolf_problem_prunned->getCovarianceBlock((*c_it)->getCapturePtr()->getFramePtr()->getPPtr(), (*c_it)->getCapturePtr()->getFramePtr()->getOPtr(), Sigma_ii, 0, 2);
        wolf_problem_prunned->getCovarianceBlock((*c_it)->getCapturePtr()->getFramePtr()->getOPtr(), (*c_it)->getCapturePtr()->getFramePtr()->getPPtr(), Sigma_ii, 2, 0);
        wolf_problem_prunned->getCovarianceBlock((*c_it)->getCapturePtr()->getFramePtr()->getOPtr(), (*c_it)->getCapturePtr()->getFramePtr()->getOPtr(), Sigma_ii, 2, 2);
        //std::cout << "Sigma_ii" << std::endl << Sigma_ii << std::endl;
        // jj
        wolf_problem_prunned->getCovarianceBlock((*c_it)->getFrameToPtr()->getPPtr(), (*c_it)->getFrameToPtr()->getPPtr(), Sigma_jj, 0, 0);
        wolf_problem_prunned->getCovarianceBlock((*c_it)->getFrameToPtr()->getPPtr(), (*c_it)->getFrameToPtr()->getOPtr(), Sigma_jj, 0, 2);
        wolf_problem_prunned->getCovarianceBlock((*c_it)->getFrameToPtr()->getOPtr(), (*c_it)->getFrameToPtr()->getPPtr(), Sigma_jj, 2, 0);
        wolf_problem_prunned->getCovarianceBlock((*c_it)->getFrameToPtr()->getOPtr(), (*c_it)->getFrameToPtr()->getOPtr(), Sigma_jj, 2, 2);
        //std::cout << "Sigma_jj" << std::endl << Sigma_jj << std::endl;
        // ij
        wolf_problem_prunned->getCovarianceBlock((*c_it)->getCapturePtr()->getFramePtr()->getPPtr(), (*c_it)->getFrameToPtr()->getPPtr(), Sigma_ij, 0, 0);
        wolf_problem_prunned->getCovarianceBlock((*c_it)->getCapturePtr()->getFramePtr()->getPPtr(), (*c_it)->getFrameToPtr()->getOPtr(), Sigma_ij, 0, 2);
        wolf_problem_prunned->getCovarianceBlock((*c_it)->getCapturePtr()->getFramePtr()->getOPtr(), (*c_it)->getFrameToPtr()->getPPtr(), Sigma_ij, 2, 0);
        wolf_problem_prunned->getCovarianceBlock((*c_it)->getCapturePtr()->getFramePtr()->getOPtr(), (*c_it)->getFrameToPtr()->getOPtr(), Sigma_ij, 2, 2);
        //std::cout << "Sigma_ij" << std::endl << Sigma_ij << std::endl;

        //jacobian
        xi = *(*c_it)->getCapturePtr()->getFramePtr()->getPPtr()->getPtr();
        yi = *((*c_it)->getCapturePtr()->getFramePtr()->getPPtr()->getPtr()+1);
        thi = *(*c_it)->getCapturePtr()->getFramePtr()->getOPtr()->getPtr();
        si = sin(thi);
        ci = cos(thi);
        xj = *(*c_it)->getFrameToPtr()->getPPtr()->getPtr();
        yj = *((*c_it)->getFrameToPtr()->getPPtr()->getPtr()+1);

        Ji << -ci,-si,-(xj-xi)*si+(yj-yi)*ci,
               si, ci,-(xj-xi)*ci-(yj-yi)*si,
                0,  0,                    -1;
        Jj <<  ci, si, 0,
              -si, ci, 0,
                0,  0, 1;
        //std::cout << "Ji" << std::endl << Ji << std::endl;
        //std::cout << "Jj" << std::endl << Jj << std::endl;

        // Measurement covariance
        Sigma_z = (*c_it)->getFeaturePtr()->getMeasurementCovariance();
        //std::cout << "Sigma_z" << std::endl << Sigma_z << std::endl;
        //std::cout << "Sigma_z.determinant() = " << Sigma_z.determinant() << std::endl;

        //std::cout << "denominador : " << std::endl << Sigma_z - (Ji * Sigma_ii * Ji.transpose() + Jj * Sigma_jj * Jj.transpose() + Ji * Sigma_ij * Jj.transpose() + Jj * Sigma_ij.transpose() * Ji.transpose()) << std::endl;
        // Information gain
        WolfScalar IG = 0.5 * log( Sigma_z.determinant() / (Sigma_z - (Ji * Sigma_ii * Ji.transpose() + Jj * Sigma_jj * Jj.transpose() + Ji * Sigma_ij * Jj.transpose() + Jj * Sigma_ij.transpose() * Ji.transpose())).determinant() );
        std::cout << "IG = " << IG << std::endl;
    }

    // SOLVING PROBLEMS
//    std::cout << "solving..." << std::endl;
//    ceres::Solver::Summary summary_all = ceres_manager_all->solve(ceres_options);
//    std::cout << summary_all.FullReport() << std::endl;
//    ceres::Solver::Summary summary_prunned = ceres_manager_prunned->solve(ceres_options);
//    std::cout << summary_prunned.FullReport() << std::endl;

    delete wolf_problem_all; //not necessary to delete anything more, wolf will do it!
    std::cout << "wolf_problem_ deleted!" << std::endl;
    delete ceres_manager_all;
    delete ceres_manager_prunned;
    std::cout << "ceres_manager deleted!" << std::endl;
    //End message
    std::cout << " =========================== END ===============================" << std::endl << std::endl;
       
    //exit
    return 0;
}
