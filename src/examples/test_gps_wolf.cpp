// Testing create and delete full wolf tree with GPS captures


//Wolf includes
#include <processor_gps.h>
#include "wolf_manager_gps.h"
#include "ceres_wrapper/ceres_manager.h"

using namespace std;


void addRealGPSMeasurements(rawgpsutils::SatellitesObs &obs, int i);

int main(int argc, char** argv)
{
    // TEMP! just to try some math using eigen
//    testMathQuaternion();
//    return 0;

    bool useCeres = true;
    bool ceresVerbose = true;
    unsigned int n_captures = 41;

    //Welcome message
    cout << endl << " ========= WOLF TREE test ===========" << endl << endl;


    // Initial x, y, z, bias
    // 4789360.65929 177175.418126 4194534.14743 0.000242545313354


    /*
     * Parameters, to be optimized
     */
    // Initial ecef position of the experiment
    StateBlock* vehicle_init_p = new StateBlock(Eigen::Vector3s(4789000, 177000, 4194000));
    StateBlock* vehicle_init_o = new StateBlock(Eigen::Vector1s::Zero());// vehicle init orientation

    // Sensor position with respect to vehicle's frame
    StateBlock* sensor_p = new StateBlock(Eigen::Vector3s(1, 0, 0), true);//::Zero()); //gps sensor position
    StateBlock* sensor_o = new StateBlock(Eigen::Vector4s::Zero(), true);   //gps sensor orientation

    //gps sensor bias
    StateBlock* sensor_bias = new StateBlock(Eigen::Vector1s::Zero());

    /*
     * GPS Sensor
     */
    SensorGPS* gps_sensor_ptr_ = new SensorGPS(sensor_p, sensor_o, sensor_bias, vehicle_init_p, vehicle_init_o);
    gps_sensor_ptr_->addProcessor(new ProcessorGPS());

    Eigen::Vector3s prior = Eigen::Vector3s(10, 10, 90*M_PI/180);
//    4789360.65929, 177175.418126, 4194534.14743
    /*
     * GPS WolfManager
     */
    WolfManagerGPS* wolf_manager_ = new WolfManagerGPS(PO_2D,                             //frame structure
                                                       nullptr,                           //_sensor_prior_ptr
                                                       prior,           //prior
                                                       Eigen::Matrix3s::Identity()*0.01,  //prior cov
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
    for(unsigned int  i=0; i < n_captures; ++i) {
        cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
        cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%  CAPTURE #" << i << "  %%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
        TimeStamp time_stamp(i);

        rawgpsutils::SatellitesObs obs;

//        obs.addPrMeasurement(rawgpsutils::PrMeasurement(1, 666, 5000, 5000, 5000));
//        obs.addPrMeasurement(rawgpsutils::PrMeasurement(2, 777, 4000, 4500, 6000));
//        obs.addPrMeasurement(rawgpsutils::PrMeasurement(3, 888, 1000, 2000, 3000));

        addRealGPSMeasurements(obs, i);
        std::cout << "added " << obs.measurements_.size() << " measurements\n";


        // Create synthetic gps capture
        CaptureGPS *cpt_ptr_ = new CaptureGPS(time_stamp, gps_sensor_ptr_, obs);

        // Add capture
        wolf_manager_->addCapture(cpt_ptr_);
        //cout << "capture added" << endl;

        // update wolf tree
        wolf_manager_->update();
        //cout << "wolf manager updated" << endl;

        if (useCeres)
        {
            ceres_manager->update();
            //cout << "ceres manager updated" << endl;

            ceres::Solver::Summary summary;

            summary = ceres_manager->solve(ceres_options);
            if (ceresVerbose)
                cout << summary.FullReport() << endl;
        }

        //wolf_manager_->getProblemPtr()->print(2);
//
        std::cout << setprecision(12);
        std::cout << "\n~~~~ RESULTS ~~~~\n";
        std::cout << "|\tinitial P: " << gps_sensor_ptr_->getInitVehiclePPtr()->getVector().transpose() << std::endl;// initial vehicle position (ecef)
        std::cout << "|\tinitial O: " << gps_sensor_ptr_->getInitVehicleOPtr()->getVector().transpose() << std::endl;// initial vehicle orientation (ecef)
        std::cout << "|\tVehicle Pose: " << wolf_manager_->getVehiclePose().transpose() << std::endl;// position of the vehicle's frame with respect to the initial pos frame
        std::cout << "|\tVehicle P (last frame): " << wolf_manager_->getProblemPtr()->getLastFramePtr()->getPPtr()->getVector().transpose() << std::endl;// position of the vehicle's frame with respect to the initial pos frame
        std::cout << "|\tVehicle O (last frame): " << wolf_manager_->getProblemPtr()->getLastFramePtr()->getOPtr()->getVector().transpose() << std::endl;// position of the vehicle's frame with respect to the initial pos frame

//        //To print all the previous frames
//        for (auto it : *(wolf_manager_->getProblemPtr()->getTrajectoryPtr()->getFrameListPtr()))
//        {
//            std::cout << "|\tVehicle P: " << it->getPPtr()->getVector().transpose() << std::endl;
//            std::cout << "|\tVehicle O: " << it->getOPtr()->getVector().transpose() << std::endl;
//        }
        std::cout << "|\tsensor P: " << gps_sensor_ptr_->getPPtr()->getVector().transpose() << std::endl;// position of the sensor with respect to the vehicle's frame
//        std::cout << "|\tsensor O (not needed):" << gps_sensor_ptr_->getOPtr()->getVector().transpose() << std::endl;// orientation of antenna is not needed, because omnidirectional
        std::cout << "|\tbias: " << gps_sensor_ptr_->getIntrinsicPtr()->getVector().transpose() << std::endl;//intrinsic parameter  = receiver time bias
        std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n\n";


        std::cin.ignore();
    }


    cout << std::endl << " ========= calling delete wolf_manager_ (should not crash) =============" << std::endl;
    delete wolf_manager_; //not necessary to delete anything more, wolf will do it!

    cout << " ========= calling delete ceres_manager "
         << ((useCeres) ? "(and now a seg fault)" : "(should not crash)   ")
         << " ==========" << std::endl;
    delete ceres_manager; //not necessary to delete anything more, wolf will do it!

    //End message
    cout << std::endl << " ================================= END =================================" << std::endl;

    //exit
    return 0;
}

void addRealGPSMeasurements(rawgpsutils::SatellitesObs &obs, int i)
{
    switch (i)
    {
        case 0:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21721495.6298, 26409387.8247, 573003.543324, 3153763.63525));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23190316.5079, 4860606.44665, -16885408.0132, 19792037.557));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21732058.6383, 25421007.324, -6415001.98836, 4230739.78945));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20846969.1402, 22341766.4761, 6695143.58254, 13194248.4467));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21767220.8807, 15879718.1232, -14384827.6997, 15839259.0549));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21222035.1976, 12072285.4853, 10288910.8024, 21282580.8394));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23619862.6735, 4525327.84989, 20248977.9349, 16503748.902));
            break;

        case 1:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21721345.135, 26409349.7003, 573035.482988, 3154075.72471));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23190161.1089, 4860774.43561, -16885217.1117, 19792160.6616));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21732008.1874, 25421064.0542, -6414982.67639, 4230427.52703));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20846855.9292, 22341605.5691, 6695178.92475, 13194501.6944));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21767113.8518, 15879915.2988, -14384836.5217, 15839043.5821));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21221971.0998, 12072178.5912, 10289159.7953, 21282521.0628));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23619815.8035, 4525226.88799, 20249161.8406, 16503544.535));
            break;

        case 2:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21721194.6161, 26409311.5721, 573067.423116, 3154387.81351));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23190005.738, 4860942.42657, -16885026.21, 19792283.7618));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21731957.7946, 25421120.7807, -6414963.36422, 4230115.2637));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20846742.7733, 22341444.6593, 6695214.26827, 13194754.9393));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21767006.9898, 15880112.4717, -14384845.3444, 15838828.1061));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21221907.004, 12072071.6987, 10289408.7882, 21282461.2816));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23619768.9004, 4525125.92805, 20249345.7445, 16503340.1645));
            break;

        case 3:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21721044.0433, 26409273.4403, 573099.363711, 3154699.90164));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23189850.3561, 4861110.41953, -16884835.3079, 19792406.8579));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21731907.3208, 25421177.5033, -6414944.05185, 4229802.99947));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20846629.5333, 22341283.7465, 6695249.61311, 13195008.1815));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21766899.8628, 15880309.6421, -14384854.1677, 15838612.6266));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21221842.8843, 12071964.808, 10289657.7809, 21282401.4959));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23619722.0693, 4525024.97006, 20249529.6465, 16503135.7905));
            break;

        case 4:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21720893.5354, 26409235.3047, 573131.30477, 3155011.9891));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23189694.9961, 4861278.41449, -16884644.4056, 19792529.9496));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21731856.798, 25421234.2222, -6414924.73929, 4229490.73434));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20846516.2603, 22341122.8308, 6695284.95926, 13195261.421));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21766792.6979, 15880506.8098, -14384862.9915, 15838397.1439));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21221778.8615, 12071857.919, 10289906.7736, 21282341.7056));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23619675.1572, 4524924.01402, 20249713.5469, 16502931.413));
            break;

        case 5:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21720742.9175, 26409197.1654, 573163.246295, 3155324.0759));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23189539.5562, 4861446.41144, -16884453.503, 19792653.0371));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21731806.4642, 25421290.9373, -6414905.42653, 4229178.46831));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20846403.0594, 22340961.9122, 6695320.30673, 13195514.6577));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21766685.7149, 15880703.9749, -14384871.816, 15838181.6578));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21221714.7837, 12071751.0316, 10290155.7662, 21282281.9108));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23619628.2672, 4524823.05994, 20249897.4454, 16502727.0319));
            break;

        case 6:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21720592.4027, 26409159.0223, 573195.188285, 3155636.16202));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23189384.0943, 4861614.41039, -16884262.6001, 19792776.1204));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21731756.1334, 25421347.6486, -6414886.11357, 4228866.20138));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20846289.9764, 22340800.9906, 6695355.65551, 13195767.8917));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21766578.7469, 15880901.1373, -14384880.6411, 15837966.1684));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21221650.7059, 12071644.146, 10290404.7587, 21282222.1115));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23619581.3061, 4524722.10782, 20250081.3421, 16502522.6473));
            break;

        case 7:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21720441.8258, 26409120.8756, 573227.13074, 3155948.24748));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23189228.5953, 4861782.41134, -16884071.697, 19792899.1994));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21731705.7605, 25421404.3561, -6414866.80042, 4228553.93356));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20846176.7885, 22340640.0662, 6695391.00561, 13196021.123));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21766471.604, 15881098.2971, -14384889.4669, 15837750.6756));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21221586.7222, 12071537.2621, 10290653.7511, 21282162.3076));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23619534.412, 4524621.15765, 20250265.2371, 16502318.2591));
            break;

        case 8:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21720291.255, 26409082.7251, 573259.073661, 3156260.33227));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23189073.0504, 4861950.41429, -16883880.7935, 19793022.2742));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21731655.4107, 25421461.0599, -6414847.48707, 4228241.66483));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20846063.6915, 22340479.1388, 6695426.35703, 13196274.3515));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21766364.585, 15881295.4543, -14384898.2932, 15837535.1795));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21221522.6794, 12071430.3798, 10290902.7434, 21282102.4992));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23619487.5019, 4524520.20943, 20250449.1303, 16502113.8674));
            break;

        case 9:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21720140.7401, 26409044.5709, 573291.017048, 3156572.41639));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23188917.6395, 4862118.41923, -16883689.8898, 19793145.3447));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21731605.0759, 25421517.7599, -6414828.17352, 4227929.3952));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20845950.5735, 22340318.2085, 6695461.70976, 13196527.5773));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21766257.563, 15881492.6089, -14384907.1201, 15837319.6801));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21221458.6056, 12071323.4993, 10291151.7357, 21282042.6863));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23619440.5589, 4524419.26317, 20250633.0217, 16501909.4722));
            break;

        case 10:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21719990.0603, 26409006.413, 573322.9609, 3156884.49984));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23188762.3356, 4862286.42617, -16883498.9858, 19793268.4109));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21731554.7611, 25421574.4561, -6414808.85977, 4227617.12467));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20845837.3536, 22340157.2752, 6695497.06381, 13196780.8004));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21766150.5711, 15881689.7608, -14384915.9477, 15837104.1773));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21221394.4918, 12071216.6205, 10291400.7278, 21281982.8688));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23619393.6468, 4524318.31887, 20250816.9113, 16501705.0735));
            break;

        case 11:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21719839.4764, 26408968.2514, 573354.905218, 3157196.58262));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23188606.7836, 4862454.43511, -16883308.0815, 19793391.4729));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21731504.4493, 25421631.1485, -6414789.54583, 4227304.85324));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20845724.0736, 22339996.339, 6695532.41918, 13197034.0207));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21766043.5921, 15881886.9101, -14384924.7758, 15836888.6712));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21221330.3761, 12071109.7433, 10291649.7199, 21281923.0468));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23619346.7187, 4524217.37651, 20251000.7991, 16501500.6712));
            break;

        case 12:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21719688.8426, 26408930.086, 573386.850001, 3157508.66474));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23188451.3797, 4862622.44605, -16883117.1769, 19793514.5306));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21731454.1215, 25421687.8372, -6414770.23169, 4226992.58091));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20845610.8597, 22339835.4, 6695567.77586, 13197287.2383));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21765936.4381, 15882084.0568, -14384933.6046, 15836673.1618));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21221266.2263, 12071002.8679, 10291898.7119, 21281863.2203));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23619299.7796, 4524116.43612, 20251184.6852, 16501296.2654));
            break;

        case 13:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21719538.3057, 26408891.9169, 573418.79525, 3157820.74618));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23188295.9548, 4862790.45898, -16882926.2721, 19793637.5841));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21731403.7596, 25421744.522, -6414750.91735, 4226680.30768));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20845497.7067, 22339674.4579, 6695603.13386, 13197540.4532));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21765829.6092, 15882281.2008, -14384942.4339, 15836457.649));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21221202.1775, 12070895.9942, 10292147.7038, 21281803.3892));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23619252.8386, 4524015.49768, 20251368.5694, 16501091.8561));
            break;

        case 14:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21719387.7459, 26408853.7441, 573450.740965, 3158132.82696));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23188140.7138, 4862958.47391, -16882735.3669, 19793760.6333));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21731353.4158, 25421801.2031, -6414731.60281, 4226368.03355));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20845384.5058, 22339513.513, 6695638.49317, 13197793.6653));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21765722.6682, 15882478.3422, -14384951.2639, 15836242.1328));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21221138.1827, 12070789.1222, 10292396.6956, 21281743.5536));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23619205.8485, 4523914.56119, 20251552.4519, 16500887.4432));
            break;

        case 15:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21719237.102, 26408815.5676, 573482.687145, 3158444.90707));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23187985.4169, 4863126.49084, -16882544.4615, 19793883.6782));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21731303.061, 25421857.8805, -6414712.28808, 4226055.75852));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20845271.3578, 22339352.5651, 6695673.8538, 13198046.8747));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21765615.8922, 15882675.481, -14384960.0945, 15836026.6134));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21221074.194, 12070682.2518, 10292645.6873, 21281683.7135));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23619158.8964, 4523813.62666, 20251736.3326, 16500683.0268));
            break;

        case 16:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21719086.5982, 26408777.3874, 573514.633792, 3158756.98651));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23187830.073, 4863294.50977, -16882353.5558, 19794006.7189));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21731252.7022, 25421914.554, -6414692.97315, 4225743.4826));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20845158.2168, 22339191.6144, 6695709.21575, 13198300.0814));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21765509.0033, 15882872.6171, -14384968.9257, 15835811.0906));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21221010.0962, 12070575.3832, 10292894.6789, 21281623.8688));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23619111.9974, 4523712.69408, 20251920.2115, 16500478.6069));
            break;

        case 17:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21718936.1013, 26408739.2034, 573546.580904, 3159069.06528));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23187674.664, 4863462.53069, -16882162.6498, 19794129.7554));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21731202.3364, 25421971.2238, -6414673.65802, 4225431.20577));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20845045.0739, 22339030.6606, 6695744.57901, 13198553.2853));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21765402.0793, 15883069.7506, -14384977.7575, 15835595.5645));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21220946.0934, 12070468.5163, 10293143.6705, 21281564.0196));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23619065.0853, 4523611.76345, 20252104.0887, 16500274.1834));
            break;

        case 18:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21718785.5145, 26408701.0157, 573578.528482, 3159381.14338));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23187519.3581, 4863630.55361, -16881971.7436, 19794252.7876));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21731152.0176, 25422027.8898, -6414654.34269, 4225118.92804));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20844931.9329, 22338869.704, 6695779.94359, 13198806.4865));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21765295.1164, 15883266.8815, -14384986.5899, 15835380.035));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21220881.9756, 12070361.6511, 10293392.6619, 21281504.1658));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23619018.1642, 4523510.83479, 20252287.964, 16500069.7565));
            break;

        case 19:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21718634.9906, 26408662.8243, 573610.476526, 3159693.22081));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23187364.0632, 4863798.57853, -16881780.837, 19794375.8155));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21731101.7227, 25422084.552, -6414635.02717, 4224806.64942));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20844818.778, 22338708.7444, 6695815.30948, 13199059.6849));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21765188.2354, 15883464.0097, -14384995.423, 15835164.5022));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21220817.8509, 12070254.7876, 10293641.6533, 21281444.3075));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23618971.2551, 4523409.90807, 20252471.8376, 16499865.3259));
            break;

        case 20:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21718484.5158, 26408624.6292, 573642.425037, 3160005.29757));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23187208.7663, 4863966.60544, -16881589.9302, 19794498.8392));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21731051.3889, 25422141.2104, -6414615.71145, 4224494.36989));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20844705.637, 22338547.782, 6695850.6767, 13199312.8806));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21765081.2044, 15883661.1354, -14385004.2566, 15834948.966));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21220753.7411, 12070147.9257, 10293890.6446, 21281384.4447));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23618924.2651, 4523308.98331, 20252655.7094, 16499660.8919));
            break;

        case 21:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21718333.9769, 26408586.4303, 573674.374013, 3160317.37367));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23187053.4743, 4864134.63435, -16881399.0231, 19794621.8586));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21731001.0851, 25422197.8651, -6414596.39553, 4224182.08947));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20844592.503, 22338386.8166, 6695886.04523, 13199566.0736));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21764974.1835, 15883858.2584, -14385013.0908, 15834733.4266));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21220689.6413, 12070041.0656, 10294139.6358, 21281324.5773));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23618877.321, 4523208.06051, 20252839.5794, 16499456.4543));
            break;

        case 22:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21718183.4061, 26408548.2277, 573706.323456, 3160629.44909));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23186898.1574, 4864302.66526, -16881208.1157, 19794744.8738));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21730950.7413, 25422254.5159, -6414577.07941, 4223869.80814));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20844479.3661, 22338225.8482, 6695921.41507, 13199819.2639));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21764867.3025, 15884055.3787, -14385021.9257, 15834517.8837));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21220625.5165, 12069934.2072, 10294388.6269, 21281264.7054));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23618830.3669, 4523107.13966, 20253023.4476, 16499252.0132));
            break;

        case 23:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21718032.7842, 26408510.0215, 573738.273364, 3160941.52385));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23186742.8555, 4864470.69817, -16881017.208, 19794867.8847));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21730900.3665, 25422311.1631, -6414557.7631, 4223557.52592));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20844366.2371, 22338064.877, 6695956.78623, 13200072.4514));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21764760.5915, 15884252.4964, -14385030.7611, 15834302.3376));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21220561.4338, 12069827.3505, 10294637.6179, 21281204.829));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23618783.4058, 4523006.22076, 20253207.314, 16499047.5686));
            break;

        case 24:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21717882.2554, 26408471.8114, 573770.223739, 3161253.59793));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23186587.5615, 4864638.73307, -16880826.3001, 19794990.8913));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21730849.9206, 25422367.8064, -6414538.44658, 4223245.24279));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20844253.0852, 22337903.9028, 6695992.15871, 13200325.6362));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21764653.7376, 15884449.6115, -14385039.5972, 15834086.7881));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21220497.318, 12069720.4955, 10294886.6088, 21281144.948));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23618736.5298, 4522905.30382, 20253391.1787, 16498843.1204));
            break;

        case 25:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21717731.7605, 26408433.5977, 573802.17458, 3161565.67135));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23186432.2816, 4864806.76997, -16880635.3919, 19795113.8937));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21730799.4828, 25422424.4459, -6414519.12987, 4222932.95877));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20844139.9152, 22337742.9257, 6696027.53251, 13200578.8182));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21764546.6936, 15884646.724, -14385048.4339, 15833871.2353));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21220433.2132, 12069613.6421, 10295135.5996, 21281085.0625));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23618689.6717, 4522804.38883, 20253575.0416, 16498638.6688));
            break;

        case 26:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21717581.2727, 26408395.3802, 573834.125888, 3161877.74409));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23186276.9877, 4864974.80887, -16880444.4834, 19795236.8919));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21730749.139, 25422481.0817, -6414499.81296, 4222620.67385));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20844026.7942, 22337581.9457, 6696062.90762, 13200831.9975));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21764439.7286, 15884843.8338, -14385057.2712, 15833655.6791));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21220369.0994, 12069506.7905, 10295384.5904, 21281025.1725));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23618642.8916, 4522703.4758, 20253758.9027, 16498434.2135));
            break;

        case 27:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21717430.6668, 26408357.1591, 573866.077662, 3162189.81617));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23186121.6887, 4865142.84976, -16880253.5746, 19795359.8857));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21730698.6902, 25422537.7137, -6414480.49585, 4222308.38803));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20843913.8163, 22337420.9627, 6696098.28405, 13201085.1741));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21764332.8037, 15885040.941, -14385066.1091, 15833440.1196));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21220305.0797, 12069399.9406, 10295633.5811, 21280965.2779));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23618596.0136, 4522602.56473, 20253942.762, 16498229.7548));
            break;

        case 28:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21717280.158, 26408318.9342, 573898.029903, 3162501.88758));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23185966.4248, 4865310.89266, -16880062.6655, 19795482.8754));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21730648.3604, 25422594.3419, -6414461.17855, 4221996.10131));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20843800.7473, 22337259.9768, 6696133.66179, 13201338.3479));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21764225.9207, 15885238.0456, -14385074.9476, 15833224.5567));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21220241.0979, 12069293.0924, 10295882.5716, 21280905.3788));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23618549.1305, 4522501.6556, 20254126.6195, 16498025.2925));
            break;

        case 29:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21717129.6501, 26408280.7056, 573929.982609, 3162813.95832));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23185811.1329, 4865478.93755, -16879871.7562, 19795605.8607));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21730598.0196, 25422650.9664, -6414441.86104, 4221683.81369));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20843687.6174, 22337098.9881, 6696169.04086, 13201591.519));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21764118.9797, 15885435.1475, -14385083.7867, 15833008.9906));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21220176.9881, 12069186.2459, 10296131.5621, 21280845.4751));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23618502.2054, 4522400.74844, 20254310.4752, 16497820.8267));
            break;

        case 30:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21716979.1333, 26408242.4732, 573961.935783, 3163126.02838));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23185655.843, 4865646.98443, -16879680.8465, 19795728.8419));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21730547.6777, 25422707.587, -6414422.54334, 4221371.52518));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20843574.4514, 22336937.9963, 6696204.42123, 13201844.6874));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21764012.1368, 15885632.2469, -14385092.6264, 15832793.4211));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21220112.8993, 12069079.4011, 10296380.5525, 21280785.5669));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23618455.2013, 4522299.84322, 20254494.3292, 16497616.3574));
            break;

        case 31:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21716828.6474, 26408204.2372, 573993.889423, 3163438.09778));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23185500.535, 4865815.03332, -16879489.9366, 19795851.8187));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21730497.1959, 25422764.2039, -6414403.22544, 4221059.23576));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20843461.3354, 22336777.0017, 6696239.80293, 13202097.853));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21763905.1288, 15885829.3435, -14385101.4668, 15832577.8482));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21220048.8796, 12068972.558, 10296629.5428, 21280725.6542));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23618408.2493, 4522198.93996, 20254678.1814, 16497411.8845));
            break;

        case 32:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21716678.1286, 26408165.9974, 574025.84353, 3163750.16651));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23185345.2601, 4865983.0842, -16879299.0264, 19795974.7913));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21730446.8521, 25422820.817, -6414383.90734, 4220746.94545));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20843348.1815, 22336616.0041, 6696275.18594, 13202351.0159));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21763798.2578, 15886026.4376, -14385110.3077, 15832362.272));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21219984.9068, 12068865.7166, 10296878.5331, 21280665.7369));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23618361.3642, 4522098.03866, 20254862.0317, 16497207.4081));
            break;

        case 33:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21716527.6107, 26408127.7539, 574057.798103, 3164062.23457));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23185189.9252, 4866151.13707, -16879108.116, 19796097.7597));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21730396.5173, 25422877.4264, -6414364.58905, 4220434.65424));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20843235.0275, 22336455.0036, 6696310.57027, 13202604.1761));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21763691.4219, 15886223.529, -14385119.1492, 15832146.6925));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21219920.844, 12068758.8769, 10297127.5232, 21280605.8151));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23618314.4841, 4521997.13931, 20255045.8803, 16497002.9282));
            break;

        case 34:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21716377.1319, 26408089.5066, 574089.753143, 3164374.30195));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23185034.5322, 4866319.19195, -16878917.2052, 19796220.7237));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21730346.1495, 25422934.0319, -6414345.27055, 4220122.36213));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20843121.9086, 22336294.0002, 6696345.95592, 13202857.3335));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21763584.5699, 15886420.6178, -14385127.9914, 15831931.1096));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21219856.7422, 12068652.0389, 10297376.5132, 21280545.8888));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23618267.661, 4521896.24191, 20255229.7272, 16496798.4447));
            break;

        case 35:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21716226.548, 26408051.2557, 574121.70865, 3164686.36867));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23184879.2203, 4866487.24882, -16878726.2942, 19796343.6836));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21730295.6947, 25422990.6337, -6414325.95186, 4219810.06912));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20843008.8036, 22336132.9939, 6696381.34288, 13203110.4882));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21763477.7609, 15886617.7039, -14385136.8342, 15831715.5234));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21219792.7425, 12068545.2026, 10297625.5032, 21280485.9579));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23618220.873, 4521795.34647, 20255413.5722, 16496593.9577));
            break;

        case 36:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21716075.9232, 26408013.001, 574153.664624, 3164998.43472));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23184723.9524, 4866655.30769, -16878535.3829, 19796466.6391));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21730245.3578, 25423047.2317, -6414306.63296, 4219497.77521));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20842895.6946, 22335971.9846, 6696416.73117, 13203363.6402));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21763370.783, 15886814.7875, -14385145.6775, 15831499.9339));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21219728.6437, 12068438.368, 10297874.4931, 21280426.0225));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23618173.8599, 4521694.45299, 20255597.4155, 16496389.4672));
            break;

        case 37:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21715925.4063, 26407974.7426, 574185.621065, 3165310.5001));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23184568.6954, 4866823.36855, -16878344.4713, 19796589.5905));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21730195.014, 25423103.8259, -6414287.31387, 4219185.48041));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20842782.5337, 22335810.9725, 6696452.12076, 13203616.7894));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21763263.748, 15887011.8683, -14385154.5215, 15831284.341));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21219664.4749, 12068331.5351, 10298123.4828, 21280366.0826));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23618126.8498, 4521593.56145, 20255781.2569, 16496184.9732));
            break;

        case 38:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21715774.9055, 26407936.4805, 574217.577972, 3165622.5648));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23184413.4055, 4866991.43141, -16878153.5594, 19796712.5375));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21730144.5362, 25423160.4164, -6414267.99458, 4218873.1847));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20842669.4307, 22335649.9574, 6696487.51168, 13203869.9359));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21763156.8501, 15887208.9466, -14385163.3661, 15831068.7448));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21219600.3631, 12068224.7039, 10298372.4725, 21280306.1381));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23618079.9587, 4521492.67188, 20255965.0966, 16495980.4756));
            break;

        case 39:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21715624.4106, 26407898.2147, 574249.535347, 3165934.62884));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23184258.0036, 4867159.49627, -16877962.6473, 19796835.4803));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21730094.1974, 25423217.0031, -6414248.67509, 4218560.8881));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20842556.3418, 22335488.9393, 6696522.90391, 13204123.0796));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21763050.0031, 15887406.0222, -14385172.2113, 15830853.1453));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21219536.3724, 12068117.8744, 10298621.4621, 21280246.1891));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23618033.0577, 4521391.78425, 20256148.9345, 16495775.9745));
            break;

        case 40:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21715473.8818, 26407859.9452, 574281.493188, 3166246.6922));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23184102.7677, 4867327.56313, -16877771.7349, 19796958.4189));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21730043.8296, 25423273.586, -6414229.3554, 4218248.5906));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20842443.1088, 22335327.9184, 6696558.29746, 13204376.2206));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21762943.0571, 15887603.0952, -14385181.0571, 15830637.5424));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21219472.3056, 12068011.0466, 10298870.4516, 21280186.2356));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23617986.1776, 4521290.89858, 20256332.7706, 16495571.4699));
            break;

        case 41:
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21715323.3469, 26407821.6719, 574313.451497, 3166558.7549));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23183947.4437, 4867495.63198, -16877580.8221, 19797081.3532));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21729993.4838, 25423330.1651, -6414210.03552, 4217936.2922));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20842329.9678, 22335166.8945, 6696593.69233, 13204629.3589));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21762836.2442, 15887800.1656, -14385189.9035, 15830421.9362));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21219408.1958, 12067904.2205, 10299119.4411, 21280126.2775));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23617939.2745, 4521190.01487, 20256516.605, 16495366.9617));
            break;

        default :
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 30, 21715172.8171, 26407783.3949, 574345.410273, 3166870.81692));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 5, 23183792.0468, 4867663.70283, -16877389.9092, 19797204.2832));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 6, 21729943.1179, 25423386.7404, -6414190.71543, 4217623.99291));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 7, 20842216.7579, 22335005.8677, 6696629.08851, 13204882.4945));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 2, 21762729.2452, 15887997.2333, -14385198.7505, 15830206.3267));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 9, 21219344.085, 12067797.3961, 10299368.4304, 21280066.3149));
            obs.addPrMeasurement(rawgpsutils::PrMeasurement( 23, 23617892.3135, 4521089.13311, 20256700.4375, 16495162.45));
            break;
    }
}
