#include <ExoSimulator.hpp>

#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string>

using namespace std;

const uint32_t nx_ = 36;
const uint32_t nu_ = 12;

string getexepath(void)
{
  char result[ PATH_MAX ];
  ssize_t count = readlink( "/proc/self/exe", result, PATH_MAX );
  string strtmp = string( result, (count > 0) ? count : 0 );
  size_t found = strtmp.find_last_of("/\\");
  return strtmp.substr(0,found);
}

void controller(const double t,
                const double* x,
                      double* u)
{
	for(uint32_t i=0; i<nu_; i++)
	{
		u[i] = 0.0;
	}
}

int main(int argc, char *argv[])
{
	// Prepare log file
	ofstream myfile;
	myfile.open("log.csv", ofstream::out | ofstream::trunc);
	myfile << "tNow";
	for(uint32_t i = 0; i<nx_; i++)
	{
		myfile <<",x" << i+1;
	}
	myfile << endl;
	myfile << fixed;
	myfile << setprecision(10);

	// Simulation options
	ExoSimulator::state_t x0(nx_,0.0);
	double t0 = 0;
	double tend = 5;
	double dt = 0.001;
	ExoSimulator::simulationOptions_t opts;
	opts.tolRel = 1.0e-6;
	opts.tolAbs = 1.0e-6;

	// Urdf
	string urdfPath = getexepath();
	urdfPath+=string("/../atalante_tom.urdf");
	cout << "URDF path: "<< urdfPath << endl;

	// Instanciate simulator
	ExoSimulator exoSim(urdfPath,controller);

	// Run simulation
	CustomTimer timer;
	tic(&timer);
	exoSim.simulate(x0,t0,tend,dt,opts);
	toc(&timer);

	// Retreive log
	cout << "Elapsed time: " << timer.dt*1.0e3 << "ms" << endl;
	cout << exoSim.log.size() << " log points" << endl;
	for(uint64_t i = 0; i<exoSim.log.size(); i++)
	{
		for(uint64_t j = 0; j<nx_; j++)
		{
			myfile << exoSim.log[i][j] <<',';
		}
		myfile << exoSim.log[i].back() << endl;
	}

	// Close log file
	myfile.close();
	return 0;
}