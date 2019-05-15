#include "ExoSimulator.hpp"

ExoSimulator::ExoSimulator(const string urdfPath,
                           function<void(const double /*t*/,
                                         const double* /*x*/,
		                                         double* /*u*/)> controller):
log(),
urdfPath_(urdfPath),
controller_(controller),
options_(),
model_(),
data_(model_)
{
	setUrdfPath(urdfPath);
}

ExoSimulator::ExoSimulator(const string urdfPath,
                           function<void(const double /*t*/,
                                         const double* /*x*/,
		                                         double* /*u*/)> controller,
                           const ExoSimulator::modelOptions_t &options):
log(),
urdfPath_(urdfPath),
controller_(controller),
options_(),
model_(),
data_(model_)
{
	setUrdfPath(urdfPath);
	setModelOptions(options);
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
	if(x0.size()!=nx_)
	{
		cout << "Error - ExoSimulator::simulate - Size of x0 (" << x0.size() << ") inconsitent with model size (" << nx_ << ")." << endl;
		return ExoSimulator::result_t::ERROR;
	}
	state_t xx0 = x0;

	if(tend<=t0)
	{
		cout << "Error - ExoSimulator::simulate - Final time (" << tend << ") is less that initial time (" << t0 << ")."<< endl;
		return ExoSimulator::result_t::ERROR;
	}
	uint64_t nPts = round((tend - t0)/dt + 1.0);


	if(nPts<2)
	{
		cout << "Error - ExoSimulator::simulate - Number of integration points is less than 2."<< endl;
		return ExoSimulator::result_t::ERROR;
	}
	log = log_t(nPts,state_t(nx_+1,0.0));

	auto rhsBind = bind(&ExoSimulator::dynamicsCL, this,
	                    placeholders::_1,
	                    placeholders::_2,
	                    placeholders::_3);	
	auto stepper = make_dense_output(options.tolAbs,
	                                 options.tolRel,
	                                 stepper_t());
	auto itBegin = make_n_step_iterator_begin(stepper, rhsBind, xx0, t0, dt, nPts-1);
	auto itEnd = make_n_step_iterator_end(stepper, rhsBind, xx0);

	uint64_t i = 0;
	double t = t0;
	data_ = pinocchio::Data(model_);
	for(auto it = itBegin; it!=itEnd; it++)
	{
		log[i][0] = t;
		copy(it->begin(), it->end(), log[i].begin()+1);
		i++;
		t+=dt;
	}
	log.shrink_to_fit();
	return ExoSimulator::result_t::SUCCESS;
}

string ExoSimulator::getUrdfPath(void)
{
	return urdfPath_;
}

ExoSimulator::result_t ExoSimulator::setUrdfPath(const string &urdfPath)
{
	urdfPath_ = urdfPath;
	pinocchio::urdf::buildModel(urdfPath,model_);
	nq_ = model_.nq-2;
	if(nq_!=12)
	{
		cout << "Error - ExoSimulator::setUrdfPath - Urdf must have 14 joints." << endl;
		return ExoSimulator::result_t::ERROR;
	}
	nx_ = nq_*2 + 12;
	nu_ = model_.nv-2;
	return ExoSimulator::result_t::SUCCESS;
}

ExoSimulator::modelOptions_t ExoSimulator::getModelOptions(void)
{
	return options_;
}

ExoSimulator::result_t ExoSimulator::setModelOptions(const ExoSimulator::modelOptions_t &options)
{
	options_ = options;
	Eigen::Map<Eigen::VectorXd> g(options_.gravity,3);
	model_.gravity = g;
	return ExoSimulator::result_t::SUCCESS;
}

void ExoSimulator::dynamicsCL(const state_t &x,
                                    state_t &xDot,
                              const double t)
{
	xDot = state_t(nx_,0.0);

	Eigen::Map<const Eigen::VectorXd> q(x.data()+6,nq_);
	Eigen::Map<const Eigen::VectorXd> dq(x.data()+nq_+12,nq_);
	Eigen::Map<Eigen::VectorXd> ddq(xDot.data()+nq_+12,nq_);
	Eigen::VectorXd u = Eigen::VectorXd::Zero(nu_);
	Eigen::VectorXd uInternal(nu_);

	controller_(t,x.data(),u.data());
	internalDynamics(q,dq,uInternal);

	u+=uInternal;

	for (uint32_t i = 0; i < nq_ + 6; i++)
	{
		xDot[i] = x[nq_+6+i];
	}

	Eigen::VectorXd q14(14);
	Eigen::VectorXd dq14(14);
	Eigen::VectorXd ddq14(14);
	Eigen::VectorXd u14(14);

	q14.head<6>() = q.head<6>();
	q14.segment<6>(7) = q.tail<6>();
	q14(6) = 0.0;
	q14(13) = 0.0;

	dq14.head<6>() = dq.head<6>();
	dq14.segment<6>(7) = dq.tail<6>();
	dq14(6) = 0.0;
	dq14(13) = 0.0;

	u14.head<6>() = u.head<6>();
	u14.segment<6>(7) = u.tail<6>();
	u14(6) = 0.0;
	u14(13) = 0.0;

	ddq14 = pinocchio::aba(model_, data_, q14, dq14, u14);

	ddq.head<6>() = ddq14.head<6>();
	ddq.tail<6>() = ddq14.segment<6>(7);
}

void ExoSimulator::internalDynamics(const Eigen::VectorXd &q,
                                    const Eigen::VectorXd &dq,
                                          Eigen::VectorXd &u)
{
	// Joint friction
	for(uint32_t i = 0; i<nu_; i++)
	{
		u(i) = -options_.frictionViscous[i]*dq(i) - options_.frictionDry[i]*saturateSoft(dq(i)/options_.dryFictionVelEps,-1.0,1.0,0.7) ;
	}

	// Joint bounds
}

void ExoSimulator::contactDynamics(const Eigen::VectorXd &q,
                                   const Eigen::VectorXd &dq,
                                         Eigen::VectorXd &Fext)
{

}

double ExoSimulator::saturateSoft(const double in,
                                  const double mi,
                                  const double ma,
                                  const double r)
{
	double uc, range, middle, bevelL, bevelXc, bevelYc, bevelStart, bevelStop, out;
	const double alpha = M_PI/8;
	const double beta = M_PI/4;

	range = ma - mi;
	middle = (ma+mi)/2;
	uc = 2*(in-middle)/range;

	bevelL = r*tan(alpha);
	bevelStart = 1-cos(beta)*bevelL;
	bevelStop = 1+bevelL;
	bevelXc = bevelStop;
	bevelYc = 1 - r;

	if(uc>=bevelStop)
	{
		out = ma;
	}
	else if(uc<=-bevelStop)
	{
		out = mi;
	}
	else if(uc<=bevelStart && uc>=-bevelStart)
	{
		out = in;
	}
	else if(uc>bevelStart)
	{
		out = sqrt(r*r - (uc-bevelXc)*(uc-bevelXc)) + bevelYc;
		out = 0.5*out*range + middle;
	}
	else if(uc<-bevelStart)
	{
		out = -sqrt(r*r - (uc+bevelXc)*(uc+bevelXc)) - bevelYc;
		out = 0.5*out*range + middle;
	}
	else
	{
		out = in;			
	}
	return out;
}
