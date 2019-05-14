#include <ExoSimulator.hpp>

#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string>

using namespace std;

void controller(const double* x,
                      double* u)
{

}

int main(int argc, char *argv[])
{
	ExoSimulator::state_t x0(36,0.0);

	double t0 = 0;
	double tend = 5;
	double dt = 0.001;

	ExoSimulator exoSim(string("/home/tom/Desktop/tom/Boulot/ExoProject/variable_assist_exo/GravityComp/atalante_tom.urdf"),controller);

	CustomTimer timer;
	tic(&timer);
	exoSim.simulate(x0,t0,tend,dt);
	toc(&timer);

	ExoSimulator::log_t log;
	log = exoSim.getLog();

	cout << "Elapsed time: " << timer.dt*1.0e3 << "ms" << endl;
	cout << log.size() << " log points" << endl;
	for(uint64_t i = 0; i<log.size(); i++)
	{
		cout << "t:" << log[i][0] << endl;
	}

	return 0;
}