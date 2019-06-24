#include <sys/types.h>
#include <pwd.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <getopt.h>
#include <string>

#include "exo_simu/core/Types.h"
#include "exo_simu/core/Utilities.h"
#include "exo_simu/core/Engine.h"
#include "exo_simu/wdc/ExoModel.h"
#include "exo_simu/wdc/ExoController.h"


using namespace exo_simu;

// =====================================================================
// ================= Defines the command and callback ==================
// =====================================================================

vectorN_t Kp = (vectorN_t(14) << 41000.0, 16000.0, 16000.0, 32000.0, 4500.0, 3500.0, 0.0,
                                 41000.0, 16000.0, 16000.0, 32000.0, 4500.0, 3500.0, 0.0).finished();
vectorN_t Kd = (vectorN_t(14) << 500.0, 160.0, 120.0, 270.0, 15.0, 20.0, 0.0, 
                                 500.0, 160.0, 120.0, 270.0, 15.0, 20.0, 0.0).finished();

void compute_command(float64_t const & t,
                     vectorN_t const & q,
                     vectorN_t const & v,
                     matrixN_t const & optoforces,
                     matrixN_t const & IMUs,
                     vectorN_t       & u)
{
    u = -(Kp.array() * q.segment<14>(7).array() + Kd.array() * v.segment<14>(6).array());
}

bool callback(float64_t const & t, 
              vectorN_t const & x)
{
    return true;
}

int main(int argc, char *argv[])
{
    // =====================================================================
    // ==================== Extract the user paramaters ====================
    // =====================================================================

    // Default argument(s)
    struct passwd *pw = getpwuid(getuid());
    const char *homedir = pw->pw_dir;
    std::string urdfPath = std::string(homedir) + std::string("/.simulation/atalante_with_patient/atalante_with_patient.urdf");
    std::string outputDirPath = std::string("/tmp/blackbox/");

    // Parsing of the user argument(s)
    const char* const short_opts = "u:o:h";
    const option long_opts[] = {
        {"urdf", required_argument, nullptr, 'u'},
        {"output", required_argument, nullptr, 'o'},
        {"help", no_argument, nullptr, 'h'},
        {nullptr, no_argument, nullptr, 0}
    };

    int opt;
    while ( (opt = getopt_long(argc, argv, short_opts, long_opts, nullptr)) != -1 )
    {
        switch (opt)
        {
        case 'u':
            urdfPath = std::string(optarg);
            break;
        case 'o':
            outputDirPath = std::string(optarg);
            break;
        case 'h': // -h or --help
        case '?': // Unrecognized option
        default:
            std::cout << "Options:" << std::endl;
            std::cout << "--urdf (-u)   : Full path of the URDF file." << std::endl;
            std::cout << "--output (-o) : Full path of the log directory." << std::endl << std::endl;
            break;
        }
    }

    // Display some information for the user
    std::cout << "URDF path: "<< urdfPath << std::endl;
    std::cout << "Log directory: "<< outputDirPath << std::endl;

    // =====================================================================
    // ============= Instantiate and configure the simulator ===============
    // =====================================================================

    // Instantiate timer
    Timer timer;

    timer.tic();

    // Instantiate and configuration the exoskeleton model
    ExoModel model;
    model.initialize(urdfPath);
    configHolder_t mdlOptions = model.getOptions();
    model.setOptions(mdlOptions);

    // Instantiate and configuration the exoskeleton controller
    ExoController controller;
    controller.initialize(compute_command);
    configHolder_t ctrlOptions = controller.getOptions();
    controller.setOptions(ctrlOptions);

    // Instantiate and configuration the simulator
    Engine simulator;
    simulator.initialize(model, controller, callback);
    configHolder_t simuOptions = simulator.getDefaultOptions();
    boost::get<float64_t>(boost::get<configHolder_t>(simuOptions.at("stepper")).at("tolRel")) = 1.0e-5;
    boost::get<float64_t>(boost::get<configHolder_t>(simuOptions.at("stepper")).at("tolAbs")) = 1.0e-4;
    boost::get<float64_t>(boost::get<configHolder_t>(simuOptions.at("contacts")).at("stiffness")) = 1e6;
    boost::get<float64_t>(boost::get<configHolder_t>(simuOptions.at("contacts")).at("damping")) = 2000.0;
    boost::get<float64_t>(boost::get<configHolder_t>(simuOptions.at("contacts")).at("dryFrictionVelEps")) = 0.01;
    boost::get<float64_t>(boost::get<configHolder_t>(simuOptions.at("contacts")).at("frictionDry")) = 5.0;
    boost::get<float64_t>(boost::get<configHolder_t>(simuOptions.at("contacts")).at("frictionViscous")) = 5.0;
    boost::get<float64_t>(boost::get<configHolder_t>(simuOptions.at("contacts")).at("transitionEps")) = 0.001;
    boost::get<vectorN_t>(simuOptions.at("gravity"))(2) = -9.81;
    // boost::get<bool>(simOpts.at("isLogControllerEnable")) = false;
    // boost::get<bool>(simOpts.at("isLogLogForceSensorsEnable")) = false;
    // boost::get<bool>(simOpts.at("isLogLogImuSensorsEnable")) = false;
    simulator.setOptions(simuOptions);

    timer.toc();
    std::cout << "Instantiation time: " << timer.dt*1.0e3 << "ms" << std::endl;

    // =====================================================================
    // ======================== Run the simulator ==========================
    // =====================================================================

    // Prepare options
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(41);
    x0(2) = 1.0;
    x0(6) = 1.0;

    float64_t tf = 5.0;
    float64_t dt = 0.001;

    // Run simulation
    timer.tic();
    simulator.simulate(x0,tf,dt);
    timer.toc();
    std::cout << "Simulation time: " << timer.dt*1.0e3 << "ms" << std::endl;

    // TODO: Write the log file
    std::cout << simulator.log.size() << " log points" << std::endl;

    std::ofstream myfile;
    myfile.open(outputDirPath + std::string("/log.csv"), std::ofstream::out | std::ofstream::trunc);
    myfile << std::fixed;
    myfile << std::setprecision(10);

    std::cout << simulator.log.size() << " log points" << std::endl;
    for(uint64_t i = 0; i<simulator.log.size(); i++)
    {
        for(uint64_t j = 0; j<simulator.log[0].size()-1; j++)
        {
            myfile << simulator.log[i][j] << ',';
        }
        myfile << simulator.log[i].back() << std::endl;
    }
    myfile.close();

    return 0;
}