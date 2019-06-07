#ifndef EXO_SIMULATOR_H
#define EXO_SIMULATOR_H

#include <string>
#include <vector>
#include <functional>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/iterator/n_step_iterator.hpp>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/aba.hpp"


using namespace std;
using namespace boost::numeric::odeint;


class ExoSimulator
{
////////////////Public typedefs//////////////////
public:

	typedef vector<double> state_t;
	typedef Eigen::Matrix<double,6,1> Vector6d;
	typedef Eigen::Matrix<double,12,1> Vector12d;

	enum class result_t : int32_t
	{
		SUCCESS = 1,
		ERROR_GENERIC = -1,
		ERROR_BAD_INPUT = -2,
		ERROR_INIT_FAILED = -3
	};

	typedef struct
	{
		double frictionViscous = 0.8;
		double frictionDry = 1.0;
		double dryFictionVelEps = 1.0e-3;
		double stiffness = 5.0e5;
		double damping = 5.0e3;
		double transitionEps = 1.0e-3;
	} contactOptions_t;

	typedef struct
	{
		Vector12d frictionViscous = (Vector12d() << 100.0,100.0,100.0,100.0,20.0,20.0,100.0,100.0,100.0,100.0,20.0,20.0).finished();
		Vector12d frictionDry = (Vector12d() << 10.0,10.0,10.0,10.0,2.0,2.0,10.0,10.0,10.0,10.0,2.0,2.0).finished();
		Vector12d boundsMin = -(Vector12d() << M_PI,M_PI,M_PI,M_PI,M_PI,M_PI,M_PI,M_PI,M_PI).finished();
		Vector12d boundsMax = (Vector12d() << M_PI,M_PI,M_PI,M_PI,M_PI,M_PI,M_PI,M_PI,M_PI).finished();
		bool boundsFromUrdf = true;
		double dryFictionVelEps = 1.0e-2;
		double boundStiffness = 5.0e5;
		double boundDamping = 5.0e2;
		double boundTransitionEps = 2.0e-3;
	} jointOptions_t;

	typedef struct
	{
		jointOptions_t joints;
		contactOptions_t contacts;
		Vector6d gravity = (Vector6d() << 0.0,0.0,-9.81,0.0,0.0,0.0).finished();
	} modelOptions_t;

	typedef struct
	{
		double tolAbs = 1.0e-6;
		double tolRel = 1.0e-6;
		bool logController = true;
		bool logOptoforces = true;
		bool logIMUs = true;
	} simulationOptions_t;

	typedef vector<state_t> log_t;

////////////////Protected typedefs//////////////////
protected:
	typedef runge_kutta_dopri5<state_t> stepper_t;

/////////////////Public methods///////////////////
public:
	//Constructor & destructor
	ExoSimulator(const string urdfPath);

	ExoSimulator(const string urdfPath,
	             const modelOptions_t &options);

	~ExoSimulator(void);

	//Simulate functions
	result_t simulate(const Eigen::VectorXd &x0,
	                  const double &t0,
	                  const double &tend,
	                  const double &dt,
	                  function<void(const double /*t*/,
                                    const Eigen::VectorXd &/*x*/,
                                    const Eigen::MatrixXd &/*optoforces*/,
                                    const Eigen::MatrixXd &/*IMUs*/,
                                          Eigen::VectorXd &/*u*/)> controller);

	result_t simulate(const Eigen::VectorXd &x0,
	                  const double &t0,
	                  const double &tend,
	                  const double &dt,
	                  function<void(const double /*t*/,
                                    const Eigen::VectorXd &/*x*/,
                                    const Eigen::MatrixXd &/*optoforces*/,
                                    const Eigen::MatrixXd &/*IMUs*/,
                                          Eigen::VectorXd &/*u*/)> controller,
	                  function<bool(const double /*t*/,
	                                const Eigen::VectorXd &/*x*/)> monitorFun);

	result_t simulate(const Eigen::VectorXd &x0,
	                  const double &t0,
	                  const double &tend,
	                  const double &dt,
	                  function<void(const double /*t*/,
                                    const Eigen::VectorXd &/*x*/,
                                    const Eigen::MatrixXd &/*optoforces*/,
                                    const Eigen::MatrixXd &/*IMUs*/,
                                          Eigen::VectorXd &/*u*/)> controller,
	                  const simulationOptions_t &simOptions);

	result_t simulate(const Eigen::VectorXd &x0,
                  const double &t0,
                  const double &tend,
                  const double &dt,
                  function<void(const double /*t*/,
                                const Eigen::VectorXd &/*x*/,
                                const Eigen::MatrixXd &/*optoforces*/,
                                const Eigen::MatrixXd &/*IMUs*/,
                                      Eigen::VectorXd &/*u*/)> controller,
	                  function<bool(const double /*t*/,
	                                const Eigen::VectorXd &/*x*/)> monitorFun,
	                  const simulationOptions_t &simOptions);

	//Accessors
	string getUrdfPath(void);
	modelOptions_t getModelOptions(void);

////////////////Protected methods/////////////////
protected:
	void setUrdfPath(const string &urdfPath);
	void setModelOptions(const modelOptions_t &options);
	bool checkCtrl(function<void(const double /*t*/,
                                 const Eigen::VectorXd &/*x*/,
                                 const Eigen::MatrixXd &/*optoforces*/,
                                 const Eigen::MatrixXd &/*IMUs*/,
                                       Eigen::VectorXd &/*u*/)> controller);

	void dynamicsCL(const state_t &x,
	                      state_t &xDot,
	                const double t);
	void internalDynamics(const Eigen::VectorXd &q,
	                      const Eigen::VectorXd &dq,
	                            Eigen::VectorXd &u);
	Vector6d contactDynamics(const int32_t &frameId);

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
	              const Eigen::VectorXd &/*x*/,
	              const Eigen::MatrixXd &/*optoforces*/,
	              const Eigen::MatrixXd &/*IMUs*/,
	                    Eigen::VectorXd &/*u*/)> controller_;
	modelOptions_t options_;
	pinocchio::Model model_;
	pinocchio::Data data_;

	const int64_t nq_;
	const int64_t ndq_;
	const int64_t nx_;
	const int64_t nu_;

	const int64_t nqFull_;
	const int64_t ndqFull_;
	const int64_t nxFull_;
	const int64_t nuFull_;

	bool tesc_;

	const vector<string> contactFramesNames_;
	const vector<string> imuFramesNames_;
	const vector<string> jointsNames_;

	vector<int32_t> contactFramesIdx_;
	vector<int32_t> imuFramesIdx_;
	vector<int32_t> jointsIdx_;
};

#endif //end of EXO_SIMULATOR_H
