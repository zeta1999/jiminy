#ifndef EXO_SIMULATOR_H
#define EXO_SIMULATOR_H

#include "ExoSimulatorUtils.hpp"

#include <vector>
#include <map>
#include <functional>
#include <iostream>
#include <iomanip>

#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/iterator/n_step_iterator.hpp>

using namespace boost::numeric::odeint;
using namespace std;

class ExoSimulator
{
// typedefs
public:
	typedef struct
	{
		double relaxCost = 50.0;
		double relaxReachLb = 5.0;
		double relaxSafeLb = 5.0;
		double backTrajHorizon = 1.0;
		double backTrajDt = 0.01;
		double backTrajAbsTol = 1.0e-6;
		double backTrajRelTol = 1.0e-6;
		double satSharpness = 0.1;
		double inf = 1e20;
	} Options;

	typedef vector<double> state_t;
	typedef runge_kutta_dopri5<state_t> stepper_t;

//Methods
public:
ExoSimulator(void);
~ExoSimulator(void);

//Public attributes
public:

//Protected attributes
protected:

//Private attributes
private:

};

#endif
