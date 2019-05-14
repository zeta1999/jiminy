#include "ExoSimulator.hpp"

ExoSimulator::ExoSimulator(const string urdfPath,
                           function<void(const double* /*x*/,
		                                         double* /*u*/)> controller):
log_(),
urdfPath_(urdfPath),
controller_(controller),
options_()
{

}

ExoSimulator::ExoSimulator(const string urdfPath,
                           function<void(const double* /*x*/,
		                                         double* /*u*/)> controller,
                           const ExoSimulator::modelOptions_t options):
log_(),
urdfPath_(urdfPath),
controller_(controller),
options_(options)
{

}

ExoSimulator::~ExoSimulator(void)
{}

ExoSimulator::result_t ExoSimulator::simulate(const state_t &x0,
                                              const double &t0,
                                              const double &tend,
                                              const double &dt)
{
	return simulate(x0,t0,tend,dt,simulationOptions_t());
}

ExoSimulator::result_t ExoSimulator::simulate(const state_t &x0,
                                              const double &t0,
                                              const double &tend,
                                              const double &dt,
                                              const simulationOptions_t &options)
{
	log_.clear();

	state_t xx0 = x0;
	uint64_t nx = x0.size();

	auto rhsBind = bind(&ExoSimulator::dynamicsCL, this,
	                    placeholders::_1,
	                    placeholders::_2,
	                    placeholders::_3);	
	auto stepper = make_dense_output(options.tolAbs,
	                                 options.tolRel,
	                                 stepper_t());
	auto itBegin = make_const_step_iterator_begin(stepper, rhsBind, xx0, t0, tend, dt);
	auto itEnd = make_const_step_iterator_end(stepper, rhsBind, xx0);

	uint64_t i = 0;
	double t = t0;
	for(auto it = itBegin; it!=itEnd; it++)
	{
		log_.push_back(state_t(nx+1));
		log_[i][0] = t;
		copy(it->begin(), it->end(), log_[i].begin()+1);
		i++;
		t+=dt;
	}
	log_.shrink_to_fit();
	return ExoSimulator::result_t::SUCCESS;
}

string ExoSimulator::getUrdfPath(void)
{
	return urdfPath_;
}

void ExoSimulator::setUrdfPath(const string &urdfPath)
{
	urdfPath_ = urdfPath;
}

ExoSimulator::modelOptions_t ExoSimulator::getModelOptions(void)
{
	return options_;
}

void ExoSimulator::setModelOptions(const ExoSimulator::modelOptions_t &options)
{
	options_ = options;
}

ExoSimulator::log_t ExoSimulator::getLog(void)
{
	return log_;
}
void ExoSimulator::dynamicsCL(const state_t &x,
                                    state_t &xDot,
                              const double t)
{
	xDot = state_t(x.size(),0.0);
}