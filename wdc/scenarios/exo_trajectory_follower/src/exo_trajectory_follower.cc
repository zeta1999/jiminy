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
#include <cmath>

#include "jiminy/core/Types.h"
#include "jiminy/core/Sensor.h"
#include "jiminy/core/Engine.h"
#include "jiminy/wdc/ExoModel.h"
#include "jiminy/wdc/ExoController.h"


using namespace jiminy;
using namespace jiminy::wdc;

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
    std::string homedir(pw->pw_dir);
    std::string urdfPath = homedir + std::string("/wdc_workspace/src/jiminy/wdc/data/atalante_with_patient/atalante_with_patient.urdf");
    std::string outputDirPath("/tmp/blackbox/");

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
    boost::get<bool>(boost::get<configHolder_t>(mdlOptions.at("telemetry")).at("enableForceSensors")) = true;
    boost::get<bool>(boost::get<configHolder_t>(mdlOptions.at("telemetry")).at("enableImuSensors")) = true;
    boost::get<bool>(boost::get<configHolder_t>(mdlOptions.at("telemetry")).at("enableEncoderSensors")) = false;

    boost::get<float64_t>(boost::get<configHolder_t>(mdlOptions.at("dynamics")).at("inertiaBodiesBiasStd")) = 0.0;
    boost::get<float64_t>(boost::get<configHolder_t>(mdlOptions.at("dynamics")).at("massBodiesBiasStd")) = 0.0;
    boost::get<float64_t>(boost::get<configHolder_t>(mdlOptions.at("dynamics")).at("centerOfMassPositionBodiesBiasStd")) = 0.0;
    boost::get<float64_t>(boost::get<configHolder_t>(mdlOptions.at("dynamics")).at("relativePositionBodiesBiasStd")) = 0.0;

    boost::get<bool>(boost::get<configHolder_t>(mdlOptions.at("dynamics")).at("enableFlexibleModel")) = false;
    boost::get<std::vector<std::string> >(boost::get<configHolder_t>(mdlOptions.at("dynamics")).at("flexibleJointsNames")) =
        std::vector<std::string>({std::string("LeftTransverseHipJoint")});
    boost::get<std::vector<vectorN_t> >(boost::get<configHolder_t>(mdlOptions.at("dynamics")).at("flexibleJointsStiffness")) =
        std::vector<vectorN_t>({(vectorN_t(3) << 1.0e5, 1.0e5, 1.0e5).finished()});
    boost::get<std::vector<vectorN_t> >(boost::get<configHolder_t>(mdlOptions.at("dynamics")).at("flexibleJointsDamping")) =
        std::vector<vectorN_t>({(vectorN_t(3) << 1.0e1, 1.0e1, 1.0e1).finished()});

    model.setOptions(mdlOptions);
    model.initialize(urdfPath);

    std::unordered_map<std::string, std::vector<std::string> > sensorsNames = model.getSensorsNames();
    configHolder_t imuSensorOptions;
    model.getSensorOptions(ImuSensor::type_, sensorsNames.at(ImuSensor::type_)[0], imuSensorOptions);
    // boost::get<bool>(imuSensorOptions.at("rawData")) = true;
    // boost::get<vectorN_t>(imuSensorOptions.at("noiseStd")) = (vectorN_t(6) << 5.0e-2, 4.0e-2, 0.0, 0.0, 0.0, 0.0).finished();
    model.setSensorsOptions(ImuSensor::type_, imuSensorOptions);
    // boost::get<vectorN_t>(imuSensorOptions.at("bias")) = (vectorN_t(6) << -8.0e-2, +9.0e-2, 0.0, 0.0, 0.0, 0.0).finished();
    // boost::get<float64_t>(imuSensorOptions.at("delay")) = 2.0e-3;
    // boost::get<uint32_t>(imuSensorOptions.at("delayInterpolationOrder")) = 0U;
    model.setSensorOptions(ImuSensor::type_, sensorsNames.at(ImuSensor::type_)[0], imuSensorOptions);

    // Instantiate and configuration the exoskeleton controller
    ExoController controller;
    configHolder_t ctrlOptions = controller.getOptions();
    boost::get<bool>(ctrlOptions.at("telemetryEnable")) = true;
    controller.setOptions(ctrlOptions);
    controller.initialize(model, compute_command);

    vectorN_t qRef = vectorN_t::Zero(model.getMotorsNames().size());
    vectorN_t dqRef = vectorN_t::Zero(qRef.size());
    float64_t hzdState = 2.0; // Right leg support
    std::vector<std::string> qRefNames;
    std::vector<std::string> dqRefNames;
    for (std::string const & motorName : removeFieldnamesSuffix(model.getMotorsNames(), "Joint"))
    {
        qRefNames.emplace_back("targetPosition" + motorName);
        dqRefNames.emplace_back("targetVelocity" + motorName);
    }
    controller.registerNewVectorEntry(qRefNames, qRef);
    controller.registerNewVectorEntry(dqRefNames, dqRef);
    controller.registerNewEntry("HzdState", hzdState);

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

    // engine.registerForceImpulse("PelvisLink", 1.5, 10.0e-3, (vector3_t() << 1.0e3, 0.0, 0.0).finished());
    // engine.registerForceImpulse("PelvisLink", 2.2, 20.0e-3, (vector3_t() << 0.0, 1.0e3, 0.0).finished());
    // auto forceFct =
    //     [](float64_t const & t,
    //        vectorN_t const & x) -> vector3_t
    //     {
    //         return (vector3_t() << 1.0e2*sin(2*M_PI*(t/0.5)), 1.0e2*cos(2*M_PI*(t/0.5)), 0.0).finished();
    //     };
    // engine.registerForceProfile("LeftSagittalHipJoint", forceFct);

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