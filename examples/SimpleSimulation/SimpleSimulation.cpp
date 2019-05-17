#include <ExoSimulator.hpp>

#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string>

using namespace std;

const uint32_t nx_ = 37;
const uint32_t nu_ = 12;

string getexepath(void)
{
  char result[ PATH_MAX ];
  ssize_t count = readlink( "/proc/self/exe", result, PATH_MAX );
  string strtmp = string( result, (count > 0) ? count : 0 );
  size_t found = strtmp.find_last_of("/\\");
  return strtmp.substr(0,found);
}

ExoSimulator::Vector12d Kp = (ExoSimulator::Vector12d() << 41000.0,16000.0,16000.0,32000.0,4500.0,3500.0,41000.0,16000.0,16000.0,32000.0,4500.0,3500.0).finished();
ExoSimulator::Vector12d Kd = (ExoSimulator::Vector12d() << 500.0,160.0,120.0,270.0,15.0,20.0,500.0,160.0,120.0,270.0,15.0,20.0).finished();

void controller(const double t,
                const Eigen::VectorXd &x,
                const Eigen::MatrixXd &optoforces,
                const Eigen::MatrixXd &IMUs,
                      Eigen::VectorXd &u)
{
	for(uint32_t i=0; i<u.rows(); i++)
	{
		const double qJt = x(i+7);
		const double dqJt = x(i+19);
		const double kp = Kp(i);
		const double kd = Kd(i);

		// u(i) = -kp*qJt - kd*dqJt;
		u(i) = 0.0;
	}
}

int main(int argc, char *argv[])
{
	// Prepare log file
	ofstream myfile;
	myfile.open("log.csv", ofstream::out | ofstream::trunc);
	myfile << fixed;
	myfile << setprecision(10);

	// Urdf
	string urdfPath = getexepath();
	urdfPath+=string("/../atalante_tom.urdf");
	cout << "URDF path: "<< urdfPath << endl;

	// Simulation options
	Eigen::VectorXd x0 = Eigen::VectorXd::Zero(nx_);
	x0(2) = 1.0;
	x0(3) = 1.0;

	double t0 = 0.0;
	double tend = 3.0;
	double dt = 0.001;

	ExoSimulator::simulationOptions_t simOpts;
	simOpts.tolRel = 1.0e-7;
	simOpts.tolAbs = 1.0e-6;
	simOpts.logController = true;

	ExoSimulator::modelOptions_t modelOpts;
	// modelOpts.gravity(2) = 0.0;

	// Instanciate simulator
	ExoSimulator exoSim(urdfPath,controller,modelOpts);

	// Run simulation
	CustomTimer timer;
	tic(&timer);
	exoSim.simulate(x0,t0,tend,dt,simOpts);
	toc(&timer);

	// Retreive log
	cout << "Elapsed time: " << timer.dt*1.0e3 << "ms" << endl;
	cout << exoSim.log.size() << " log points" << endl;
	for(uint64_t i = 0; i<exoSim.log.size(); i++)
	{
		for(uint64_t j = 0; j<exoSim.log[0].size(); j++)
		{
			myfile << exoSim.log[i][j] <<',';
		}
		myfile << exoSim.log[i].back() << endl;
	}

	// Close log file
	myfile.close();
	return 0;
}