#include <sys/types.h>
#include <pwd.h>
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <getopt.h>
#include <string>
#include <time.h>

#include "exo_simu/core/Types.h"
#include "exo_simu/core/Utilities.h"
#include "exo_simu/core/Sensor.h"
#include "exo_simu/core/Engine.h"
#include "exo_simu/wdc/ExoModel.h"
#include "exo_simu/wdc/ExoController.h"


using namespace exo_simu;

// =====================================================================
// ================= Defines the command and callback ==================
// =====================================================================

rowN_t Kp = (rowN_t(12) << 41000.0, 16000.0, 16000.0, 32000.0, 4500.0, 3500.0,
                           41000.0, 16000.0, 16000.0, 32000.0, 4500.0, 3500.0).finished();
rowN_t Kd = (rowN_t(12) << 500.0, 160.0, 120.0, 270.0, 15.0, 20.0, 
                           500.0, 160.0, 120.0, 270.0, 15.0, 20.0).finished();

void compute_command(float64_t const & t,
                     vectorN_t const & q,
                     vectorN_t const & v,
                     matrixN_t const & forceSensorsData,
                     matrixN_t const & imuSensorsData,
                     matrixN_t const & encoderSensorsData,
                     vectorN_t       & u)
{
    u = -(Kp.array() * encoderSensorsData.row(0).array() + Kd.array() * encoderSensorsData.row(1).array());
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
    // ============ Instantiate and configure the simulation ===============
    // =====================================================================

    // Instantiate timer
    Timer timer;

    timer.tic();

    // Instantiate and configuration the exoskeleton model
    ExoModel model;
    configHolder_t mdlOptions = model.getOptions();
    boost::get<bool>(boost::get<configHolder_t>(mdlOptions.at("telemetry")).at("enableForceSensors")) = false;
    boost::get<bool>(boost::get<configHolder_t>(mdlOptions.at("telemetry")).at("enableImuSensors")) = true; 
    boost::get<bool>(boost::get<configHolder_t>(mdlOptions.at("telemetry")).at("enableEncoderSensors")) = false;
    model.setOptions(mdlOptions);
    model.initialize(urdfPath);
    std::map<std::string, std::vector<std::string> > sensorsNames = model.getSensorsNames();
    configHolder_t imuSensorOptions = model.getSensorOptions(ImuSensor::type_, sensorsNames.at(ImuSensor::type_)[0]);
    boost::get<bool>(imuSensorOptions.at("rawData")) = true;
    boost::get<vectorN_t>(imuSensorOptions.at("noiseStd")) = (vectorN_t(6) << 5.0e-2, 4.0e-2, 0.0, 0.0, 0.0, 0.0).finished();
    model.setSensorsOptions(ImuSensor::type_, imuSensorOptions);
    boost::get<vectorN_t>(imuSensorOptions.at("bias")) = (vectorN_t(6) << -8.0e-2, +9.0e-2, 0.0, 0.0, 0.0, 0.0).finished();
    boost::get<float64_t>(imuSensorOptions.at("delay")) = 2.0e-3;
    boost::get<uint32_t>(imuSensorOptions.at("delayInterpolationOrder")) = 0U;
    model.setSensorOptions(ImuSensor::type_, sensorsNames.at(ImuSensor::type_)[0], imuSensorOptions);

    // Instantiate and configuration the exoskeleton controller
    ExoController controller;
    configHolder_t ctrlOptions = controller.getOptions();
    boost::get<bool>(ctrlOptions.at("telemetryEnable")) = false;
    controller.setOptions(ctrlOptions);
    controller.initialize(model, compute_command);

    // Instantiate and configuration the engine
    Engine engine;
    configHolder_t simuOptions = engine.getOptions();
    boost::get<bool>(boost::get<configHolder_t>(simuOptions.at("telemetry")).at("enableConfiguration")) = true;
    boost::get<bool>(boost::get<configHolder_t>(simuOptions.at("telemetry")).at("enableVelocity")) = true;
    boost::get<bool>(boost::get<configHolder_t>(simuOptions.at("telemetry")).at("enableAcceleration")) = true;
    boost::get<bool>(boost::get<configHolder_t>(simuOptions.at("telemetry")).at("enableCommand")) = true;
    boost::get<bool>(boost::get<configHolder_t>(simuOptions.at("telemetry")).at("enableEnergy")) = true;
    boost::get<vectorN_t>(boost::get<configHolder_t>(simuOptions.at("world")).at("gravity"))(2) = -9.81;
    boost::get<std::string>(boost::get<configHolder_t>(simuOptions.at("stepper")).at("solver")) = std::string("runge_kutta_dopri5");
    boost::get<float64_t>(boost::get<configHolder_t>(simuOptions.at("stepper")).at("tolRel")) = 1.0e-5;
    boost::get<float64_t>(boost::get<configHolder_t>(simuOptions.at("stepper")).at("tolAbs")) = 1.0e-4;
    boost::get<float64_t>(boost::get<configHolder_t>(simuOptions.at("stepper")).at("dtMax")) = 3.0e-3;
    boost::get<int32_t>(boost::get<configHolder_t>(simuOptions.at("stepper")).at("iterMax")) = 100000U; // -1 for infinity
    boost::get<float64_t>(boost::get<configHolder_t>(simuOptions.at("stepper")).at("sensorsUpdatePeriod")) = 0.0; //1.0e-3;
    boost::get<float64_t>(boost::get<configHolder_t>(simuOptions.at("stepper")).at("controllerUpdatePeriod")) = 0.0; //1.0e-3;
    boost::get<int32_t>(boost::get<configHolder_t>(simuOptions.at("stepper")).at("randomSeed")) = 0U; // Use time(nullptr) for random seed.
    boost::get<float64_t>(boost::get<configHolder_t>(simuOptions.at("contacts")).at("stiffness")) = 1e6;
    boost::get<float64_t>(boost::get<configHolder_t>(simuOptions.at("contacts")).at("damping")) = 2000.0;
    boost::get<float64_t>(boost::get<configHolder_t>(simuOptions.at("contacts")).at("dryFrictionVelEps")) = 0.01;
    boost::get<float64_t>(boost::get<configHolder_t>(simuOptions.at("contacts")).at("frictionDry")) = 5.0;
    boost::get<float64_t>(boost::get<configHolder_t>(simuOptions.at("contacts")).at("frictionViscous")) = 5.0;
    boost::get<float64_t>(boost::get<configHolder_t>(simuOptions.at("contacts")).at("transitionEps")) = 0.001;
    engine.setOptions(simuOptions);
    engine.initialize(model, controller, callback);

    timer.toc();

    // =====================================================================
    // ======================= Run the simulation ==========================
    // =====================================================================

    // Prepare options
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(41);
    x0(2) = 1.0;
    x0(6) = 1.0;
    float64_t tf = 3.0;

    // Run simulation
    timer.tic();
    engine.simulate(x0,tf);
    timer.toc();
    std::cout << "Simulation time: " << timer.dt*1.0e3 << "ms" << std::endl;

    // Write the log file
    std::vector<std::string> header;
    matrixN_t log;
    engine.getLogData(header, log);
    std::cout << log.rows() << " log points" << std::endl;
    engine.writeLogTxt(outputDirPath + std::string("/log.txt"));
    engine.writeLogBinary(outputDirPath + std::string("/log.data"));

    return 0;
}