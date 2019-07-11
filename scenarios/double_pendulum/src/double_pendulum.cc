// A simple test case: simulation of a double inverted pendulum.
// There are no contact forces.
// This simulation checks the overall simulator sanity (i.e. conservation of energy) and genericity (working
// with something that is not an exoskeleton).

#include <sys/types.h>
#include <pwd.h>
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <getopt.h>
#include <string>

#include "exo_simu/core/Types.h"
#include "exo_simu/core/Utilities.h"
#include "exo_simu/core/Engine.h"
#include "exo_simu/core/AbstractController.h"


using namespace exo_simu;

// Controller object.
class PendulumController : public AbstractController
{
protected:
    typedef std::function<void(float64_t const & /*t*/,
                               vectorN_t const & /*q*/,
                               vectorN_t const & /*v*/,
                               matrixN_t const & /*forceSensorsData*/,
                               matrixN_t const & /*imuSensorsData*/,
                               matrixN_t const & /*encoderSensorsData*/,
                               vectorN_t       & /*u*/)> commandFct_t;

public:
    PendulumController(void)
    {
        isInitialized_ = true;
    }
    ~PendulumController(void)
    {}

    void updateTelemetry(void)
    {
    }

    result_t configureTelemetry(std::shared_ptr<TelemetryData> const & telemetryData)
    {
    }

    void compute_command(Model     const & model,
                         float64_t const & t,
                         vectorN_t const & q,
                         vectorN_t const & v,
                         vectorN_t       & u)
    {
        // No controller: energy should be preserved.
        u = vectorN_t::Zero(2);
    }

    void internalDynamics(Model     const & model,
                          float64_t const & t,
                          vectorN_t const & q,
                          vectorN_t const & v,
                          vectorN_t       & u)
    {
        u = vectorN_t::Zero(2);
    }
};


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

    // Set URDF and log output.
    struct passwd *pw = getpwuid(getuid());
    const char *homedir = pw->pw_dir;
    std::string urdfPath = std::string(homedir) + std::string("/wdc_workspace/src/exo_simu/examples/data/double_pendulum_rigid.urdf");
    std::string outputDirPath = std::string("/tmp/blackbox/");

    // =====================================================================
    // ============ Instantiate and configure the simulation ===============
    // =====================================================================

    // Instantiate timer
    Timer timer;

    timer.tic();

    // Instantiate and configuration the exoskeleton model
    std::vector<std::string> contacts;
    std::vector<std::string> jointNames;
    jointNames.push_back("PendulumJoint");
    jointNames.push_back("SecondPendulumJoint");

    Model model;
    configHolder_t mdlOptions = model.getOptions();
    boost::get<bool>(boost::get<configHolder_t>(mdlOptions.at("joints")).at("boundsFromUrdf")) = true;
    model.setOptions(mdlOptions);
    model.initialize(urdfPath, contacts, jointNames, false);
    model.setOptions(mdlOptions);

    // Instantiate and configuration the controller
    PendulumController controller;

    // Instantiate and configuration the engine
    Engine engine;
    configHolder_t simuOptions = engine.getDefaultOptions();
    boost::get<bool>(boost::get<configHolder_t>(simuOptions.at("telemetry")).at("logConfiguration")) = true;
    boost::get<bool>(boost::get<configHolder_t>(simuOptions.at("telemetry")).at("logVelocity")) = true;
    boost::get<bool>(boost::get<configHolder_t>(simuOptions.at("telemetry")).at("logAcceleration")) = true;
    boost::get<bool>(boost::get<configHolder_t>(simuOptions.at("telemetry")).at("logCommand")) = true;
    boost::get<vectorN_t>(boost::get<configHolder_t>(simuOptions.at("world")).at("gravity"))(2) = -9.81;
    boost::get<float64_t>(boost::get<configHolder_t>(simuOptions.at("stepper")).at("tolRel")) = 1.0e-5;
    boost::get<float64_t>(boost::get<configHolder_t>(simuOptions.at("stepper")).at("tolAbs")) = 1.0e-4;
    boost::get<float64_t>(boost::get<configHolder_t>(simuOptions.at("stepper")).at("sensorsUpdatePeriod")) = 1.0e-3;
    boost::get<float64_t>(boost::get<configHolder_t>(simuOptions.at("stepper")).at("controllerUpdatePeriod")) = 1.0e-3;
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
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(4);
    x0(1) = 0.1;
    float64_t tf = 3.0;

    // Run simulation
    timer.tic();
    engine.simulate(x0, tf);
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
