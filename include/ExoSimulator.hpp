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
#include <boost/numeric/odeint/iterator/n_step_iterator.hpp>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/aba.hpp"

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
		double frictionViscous = 0.8;
		double frictionDry = 1.0;
		double dryFictionVelEps = 1.0e-3;
		double stiffness = 5.0e5;
		double damping = 5.0e3;
	} contactOptions_t;

	typedef struct
	{
		double frictionViscous[12] = {100,100,100,100,20,20,100,100,100,100,20,20};
		double frictionDry[12] = {10,10,10,10,2,2,10,10,10,10,2,2};
		double dryFictionVelEps = 1.0e-3;
		contactOptions_t contact;
		double gravity[3] = {0,0,-9.81};

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
	             function<void(const double /*t*/,
	                           const double* /*x*/,
	                                 double* /*u*/)> controller);
	ExoSimulator(const string urdfPath,
	             function<void(const double /*t*/,
	                           const double* /*x*/,
	                                 double* /*u*/)> controller,
	             const modelOptions_t &options);
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
	ExoSimulator::result_t setUrdfPath(const string &urdfPath);

	modelOptions_t getModelOptions(void);
	ExoSimulator::result_t setModelOptions(const modelOptions_t &options);
////////////////Protected methods/////////////////
protected:
void dynamicsCL(const state_t &x,
                      state_t &xDot,
                const double t);
void internalDynamics(const Eigen::VectorXd &q,
                      const Eigen::VectorXd &dq,
                            Eigen::VectorXd &u);
void contactDynamics(const Eigen::VectorXd &q,
                     const Eigen::VectorXd &dq,
                           Eigen::VectorXd &Fext);
double saturateSoft(const double in,
                    const double mi,
                    const double ma,
                    const double r);

////////////////Public attributes/////////////////
public:
	log_t log;

//////////////Protected attributes////////////////
protected:
	string urdfPath_;
	function<void(const double /*t*/,
	              const double* /*x*/,
	                    double* /*u*/)> controller_;
	modelOptions_t options_;
	pinocchio::Model model_;
   pinocchio::Data data_;
   uint32_t nx_;
   uint32_t nu_;
   uint32_t nq_;
};

#endif //end of #ifndef EXO_SIMULATOR_sH
