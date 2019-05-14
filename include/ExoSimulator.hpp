#ifndef EXO_SIMULATOR_H
#define EXO_SIMULATOR_H

#include "ExoSimulatorUtils.hpp"

#include <vector>
#include <map>
#include <functional>
#include <iostream>
#include <iomanip>
#include <string>

#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/iterator/const_step_iterator.hpp>
// #include <boost/numeric/odeint/iterator/n_step_iterator.hpp>

using namespace boost::numeric::odeint;
using namespace std;

class ExoSimulator
{
////////////////Public typedefs//////////////////
public:

	typedef vector<double> state_t;

	enum class result_t : int32_t
	{
		SUCCESS = 1,
		ERROR = -1
	};

	typedef struct
	{
		double jointFriction = 0;
	} modelOptions_t;

	typedef struct
	{
		double tolAbs = 1.0e-6;
		double tolRel = 1.0e-6;
	} simulationOptions_t;

	typedef vector<state_t> log_t;

////////////////Protected typedefs//////////////////
protected:
	typedef runge_kutta_dopri5<state_t> stepper_t;

/////////////////Public methods///////////////////
public:
	//Constructor & destructor
	ExoSimulator(const string urdfPath,
	             function<void(const double* /*x*/,
	                                 double* /*u*/)> controller);
	ExoSimulator(const string urdfPath,
	             function<void(const double* /*x*/,
	                                 double* /*u*/)> controller,
	             const modelOptions_t options);
	~ExoSimulator(void);

	//Functions
	result_t simulate(const state_t &x0,
	                  const double &t0,
	                  const double &tend,
	                  const double &dt);

	result_t simulate(const state_t &x0,
	                  const double &t0,
	                  const double &tend,
	                  const double &dt,
	                  const simulationOptions_t &options);
	//Accessors
	string getUrdfPath(void);
	void setUrdfPath(const string &urdfPath);

	modelOptions_t getModelOptions(void);
	void setModelOptions(const modelOptions_t &options);

	log_t getLog(void);
////////////////Protected methods/////////////////
protected:
void dynamicsCL(const state_t &x,
                      state_t &xDot,
                const double t);

////////////////Public attributes/////////////////
public:
	log_t log_;

//////////////Protected attributes////////////////
protected:
	string urdfPath_;

	function<void(const double* /*x*/,
	                    double* /*u*/)> controller_;
	modelOptions_t options_;
};

#endif //end of #ifndef EXO_SIMULATOR_sH
