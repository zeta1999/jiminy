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

#include "exo_simu/engine/ExoSimulator.hpp"
#include "exo_simu/engine/ExoSimulatorUtils.hpp"

using namespace std;
using namespace exo_simu;

const uint32_t nx_ = 37;
const uint32_t nu_ = 12;

ExoSimulator::Vector12d Kp = (ExoSimulator::Vector12d() << 41000.0, 16000.0, 16000.0, 32000.0, 4500.0, 3500.0,
                                                           41000.0, 16000.0, 16000.0, 32000.0, 4500.0, 3500.0).finished();
ExoSimulator::Vector12d Kd = (ExoSimulator::Vector12d() << 500.0, 160.0, 120.0, 270.0, 15.0, 20.0, 
                                                           500.0, 160.0, 120.0, 270.0, 15.0, 20.0).finished();

void controller(const double t,
                const Eigen::VectorXd &x,
                const Eigen::MatrixXd &optoforces,
                const Eigen::MatrixXd &IMUs,
                      Eigen::VectorXd &u)
{
    // u = ExoSimulator::Vector12d::Zero();
    u = -(Kp.array()*x.segment<12>(7).array() + Kd.array()*x.segment<12>(19).array());
}

bool monitor(const double t,
             const Eigen::VectorXd &x)
{
    if(t<=2.5)
        return true;
    else
        return false;
}

int main(int argc, char *argv[])
{
    // Default argument(s)
    struct passwd *pw = getpwuid(getuid());
    const char *homedir = pw->pw_dir;
    string urdfPath = string(homedir) + string("/.simulation/atalante_with_patient/atalante_with_patient.urdf");
    string outputDirPath = string("/tmp/blackbox/");

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
            urdfPath = string(optarg);
            break;
        case 'o':
            outputDirPath = string(optarg);
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
    cout << "URDF path: "<< urdfPath << endl;
    cout << "Log directory: "<< outputDirPath << endl;

    // Prepare timer
    CustomTimer timer;

    // Prepare log file
    ofstream myfile;
    myfile.open(outputDirPath + string("/log.csv"), ofstream::out | ofstream::trunc);
    myfile << fixed;
    myfile << setprecision(10);

    // Prepare options
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(nx_);
    x0(2) = 1.0;
    x0(3) = 1.0;

    double t0 = 0.0;
    double tend = 3.0;
    double dt = 0.001;

    ExoSimulator::simulationOptions_t simOpts;
    simOpts.tolRel = 1.0e-7;
    simOpts.tolAbs = 1.0e-6;
    simOpts.logController = false;
    simOpts.logOptoforces = false;
    simOpts.logIMUs = false;

    ExoSimulator::modelOptions_t modelOpts;
    // modelOpts.gravity(2) = 9.81;

    // Instanciate simulator
    tic(&timer);
    ExoSimulator exoSim(urdfPath,modelOpts);
    toc(&timer);
    cout << "Instanciation time: " << timer.dt*1.0e3 << "ms" << endl;

    // Run simulation
    tic(&timer);
    exoSim.simulate(x0,t0,tend,dt,controller,monitor,simOpts);
    toc(&timer);
    cout << "Simulation time: " << timer.dt*1.0e3 << "ms" << endl;

    // Retreive log
    cout << exoSim.log.size() << " log points" << endl;
    for(uint64_t i = 0; i<exoSim.log.size(); i++)
    {
        for(uint64_t j = 0; j<exoSim.log[0].size()-1; j++)
        {
            myfile << exoSim.log[i][j] << ',';
        }
        myfile << exoSim.log[i].back() << endl;
    }

    // Close log file
    myfile.close();
    return 0;
}