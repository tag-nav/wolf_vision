// Testing create and delete full wolf tree with GPS captures


//Wolf includes
#include <processor_gps.h>
#include "wolf_manager_gps.h"
#include "ceres_wrapper/ceres_manager.h"

using namespace std;

int main(int argc, char** argv)
{
    bool useCeres = false;
    unsigned int n_captures = 5;

    //Welcome message
    cout << endl << " ========= WOLF TREE test ===========" << endl << endl;

    /*
     * Parameters, to be optimized
     */
    StateBlock* sensor_p = new StateBlock(Eigen::Vector3s::Zero()); //gps sensor position
    sensor_p->fix(); // TODO only for now, to semplify things
    StateBlock* sensor_o = new StateBlock(Eigen::Vector4s::Zero(), ST_QUATERNION);   //gps sensor orientation
    sensor_o->fix(); //orientation is fixed, because antenna omnidirectional, so is not going to be optimized
    StateBlock* sensor_bias = new StateBlock(Eigen::Vector1s::Zero());    //gps sensor bias
    // TODO Should this 2 supplementary blocks go in the sensor?
    StateBlock* vehicle_init_p = new StateBlock(Eigen::Vector3s::Zero());    //vehicle init position
    StateBlock* vehicle_init_o = new StateBlock(Eigen::Vector4s::Zero(), ST_QUATERNION);// vehicle init orientation

    /*
     * GPS Sensor
     */
    SensorGPS* gps_sensor_ptr_ = new SensorGPS(sensor_p, sensor_o, sensor_bias, vehicle_init_p, vehicle_init_o);
    gps_sensor_ptr_->addProcessor(new ProcessorGPS());


    /*
     * GPS WolfManager
     */
    WolfManagerGPS* wolf_manager_ = new WolfManagerGPS(PO_3D,                             //frame structure
                                                       nullptr,                           //_sensor_prior_ptr
                                                       Eigen::Vector7s::Zero(),           //prior
                                                       Eigen::Matrix7s::Identity()*0.01,  //prior cov
                                                       5,                                 //window size
                                                       1);                                //time for new keyframe


    /*
     * Ceres wrapper
     */
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    ceres_options.max_num_iterations = 1e4;
    ceres::Problem::Options problem_options;
    problem_options.cost_function_ownership = ceres::TAKE_OWNERSHIP;
    problem_options.loss_function_ownership = ceres::TAKE_OWNERSHIP;
    problem_options.local_parameterization_ownership = ceres::TAKE_OWNERSHIP;
    CeresManager* ceres_manager = new CeresManager(wolf_manager_->getProblemPtr(), problem_options);


    wolf_manager_->addSensor(gps_sensor_ptr_);


    /*
     * Data Captures
     */
    for(unsigned int  i=0; i < n_captures; ++i)
    {
        cout << "%%%%%%%%%%%%%%%%%%%%%% CAPTURE #" << i << endl;
        TimeStamp time_stamp(i);


        std::vector<ObsData> raw_data;

        WolfScalar pr(666);

        raw_data.push_back(ObsData("sat_1", TimeStamp(10, 3), pr)); pr+=111;
        raw_data.push_back(ObsData("sat_2", TimeStamp(11, 3), pr)); pr+=111;
        raw_data.push_back(ObsData("sat_3", TimeStamp(12, 3), pr));


        // Create synthetic gps capture
        CaptureGPS* cpt_ptr_ = new CaptureGPS(time_stamp, gps_sensor_ptr_, raw_data);

        // Add capture
        wolf_manager_->addCapture(cpt_ptr_);
        //cout << "capture added" << endl;

        // update wolf tree
        wolf_manager_->update();
        //cout << "wolf manager updated" << endl;

        if(useCeres)
        {
            ceres_manager->update();
            //cout << "ceres manager updated" << endl;

            ceres::Solver::Summary summary;

            summary = ceres_manager->solve(ceres_options);
            cout << summary.FullReport() << endl;
        }
    }


    cout << std::endl << " ========= calling delete wolf_manager_ (should not crash) =============" << std::endl;
    delete wolf_manager_; //not necessary to delete anything more, wolf will do it!

    cout << std::endl << " ========= calling delete ceres_manager (and now a seg fault) ==========" << std::endl;
    delete ceres_manager; //not necessary to delete anything more, wolf will do it!

    //End message
    cout << " =========================== END ===============================" << std::endl;

    //exit
    return 0;
}

