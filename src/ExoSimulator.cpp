#include "ExoSimulator.hpp"

ExoSimulator::ExoSimulator(const string urdfPath,
                           function<void(const double /*t*/,
                                         const Eigen::VectorXd &/*x*/,
		                                         Eigen::VectorXd &/*u*/)> controller):
log(),
urdfPath_(urdfPath),
controller_(controller),
options_(),
model_(),
data_(model_),
nq_(19),
ndq_(18),
nx_(nq_+ndq_),
nu_(12),
nqFull_(21),
ndqFull_(20),
nxFull_(nq_+ndq_),
nuFull_(14),
tesc_(true)
{
	setUrdfPath(urdfPath);
	modelOptions_t options;
	setModelOptions(options);
	checkCtrl();
}

ExoSimulator::ExoSimulator(const string urdfPath,
                           function<void(const double /*t*/,
                                         const Eigen::VectorXd &/*x*/,
		                                         Eigen::VectorXd &/*u*/)> controller,
                           const ExoSimulator::modelOptions_t &options):
log(),
urdfPath_(urdfPath),
controller_(controller),
options_(),
model_(),
data_(model_),
nq_(19),
ndq_(18),
nx_(nq_+ndq_),
nu_(12),
nqFull_(21),
ndqFull_(20),
nxFull_(nq_+ndq_),
nuFull_(ndqFull_),
tesc_(true)
{
	setUrdfPath(urdfPath);
	setModelOptions(options);
	checkCtrl();
}

ExoSimulator::~ExoSimulator(void)
{}

ExoSimulator::result_t ExoSimulator::simulate(const Eigen::VectorXd &x0,
                                              const double &t0,
                                              const double &tend,
                                              const double &dt)
{
	return simulate(x0,t0,tend,dt,simulationOptions_t());
}

ExoSimulator::result_t ExoSimulator::simulate(const Eigen::VectorXd &x0,
                                              const double &t0,
                                              const double &tend,
                                              const double &dt,
                                              const simulationOptions_t &options)
{
	if(!tesc_)
	{
		cout << "Error - ExoSimulator::simulate - Initialization failed, will not run simulation" << endl;
		return ExoSimulator::result_t::ERROR_INIT_FAILED;		
	}

	if(x0.rows()!=nx_)
	{
		cout << "Error - ExoSimulator::simulate - Size of x0 (" << x0.size() << ") inconsitent with model size (" << nx_ << ")." << endl;
		return ExoSimulator::result_t::ERROR_BAD_INPUT;
	}
	state_t xx0(x0.data(),x0.data()+x0.size());

	if(tend<=t0)
	{
		cout << "Error - ExoSimulator::simulate - Final time (" << tend << ") is less that initial time (" << t0 << ")."<< endl;
		return ExoSimulator::result_t::ERROR_BAD_INPUT;
	}
	uint64_t nPts = round((tend - t0)/dt + 1.0);


	if(nPts<2)
	{
		cout << "Error - ExoSimulator::simulate - Number of integration points is less than 2."<< endl;
		return ExoSimulator::result_t::ERROR_GENERIC;
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

void ExoSimulator::setUrdfPath(const string &urdfPath)
{
	urdfPath_ = urdfPath;
	pinocchio::urdf::buildModel(urdfPath,pinocchio::JointModelFreeFlyer(),model_);

	if(model_.nq!=nqFull_ || model_.nv!=ndqFull_)
	{
		cout << "Error - ExoSimulator::setUrdfPath - Urdf not recognized." << endl;
		tesc_ = false;
	}
}

ExoSimulator::modelOptions_t ExoSimulator::getModelOptions(void)
{
	return options_;
}

void ExoSimulator::setModelOptions(const ExoSimulator::modelOptions_t &options)
{
	options_ = options;
	model_.gravity = options.gravity;
}

void ExoSimulator::dynamicsCL(const state_t &x,
                                    state_t &xDot,
                              const double t)
{
	xDot.resize(nx_);

	Eigen::Map<const Eigen::VectorXd> xEig(x.data(),nx_);
	Eigen::Map<Eigen::VectorXd> xDotEig(xDot.data(),nx_);
	Eigen::Map<const Eigen::VectorXd> q(x.data(),nq_);
	Eigen::Map<const Eigen::VectorXd> dq(x.data()+nq_,ndq_);
	Eigen::Map<Eigen::VectorXd> ddq(xDot.data()+nq_,ndq_);
	Eigen::VectorXd u = Eigen::VectorXd::Zero(nu_);
	Eigen::VectorXd uWithFF = Eigen::VectorXd::Zero(ndq_);
	Eigen::VectorXd uInternal = Eigen::VectorXd::Zero(ndq_);

	xDotEig.head(3) = dq.head(3);
	Eigen::Vector3d omega = dq.segment(3,3);
	Eigen::Vector4d quat = q.segment(3,4);
	quat.normalize();
	Eigen::Matrix<double,4,4> quat2quatDot;
	quat2quatDot << 0        , omega(0), omega(1), omega(2),
	                -omega(0),        0,-omega(2), omega(1),
	                -omega(1), omega(2),        0,-omega(0),
	                -omega(2), -omega(1), omega(0),       0;
	quat2quatDot*=(-0.5);
	Eigen::Vector4d quatDot = quat2quatDot*quat;
	xDotEig.segment(3,4) = quatDot;
	xDotEig.segment(7,nq_-7) = dq.tail(ndq_-6);

	controller_(t,xEig,u);
	uWithFF.tail(ndq_-6) = u;

	internalDynamics(q,dq,uInternal);
	uWithFF+=uInternal;

	// Put quaternion in [x y z w] order
	Eigen::VectorXd qPinocchio = q;
	qPinocchio.segment(3,3) = quat.tail<3>();
	qPinocchio(6) = quat(0);

	// Specific to wandercraft urdf
	Eigen::VectorXd qFull(nqFull_);
	Eigen::VectorXd dqFull(ndqFull_);
	Eigen::VectorXd ddqFull(ndqFull_);
	Eigen::VectorXd uFull = Eigen::VectorXd::Zero(nuFull_);

	qFull.head<13>() = qPinocchio.head<13>();
	qFull.segment<6>(14) = qPinocchio.tail<6>();
	qFull(13) = 0.0;
	qFull(20) = 0.0;

	dqFull.head<12>() = dq.head<12>();
	dqFull.segment<6>(13) = dq.tail<6>();
	dqFull(12) = 0.0;
	dqFull(19) = 0.0;

	uFull.head<12>() = uWithFF.head<12>();
	uFull.segment<6>(13) = uWithFF.tail<6>();
	uFull(12) = 0.0;
	uFull(19) = 0.0;

	ddqFull = pinocchio::aba(model_, data_, qFull, dqFull, uFull);

	ddq.head<12>() = ddqFull.head<12>();
	ddq.tail<6>() = ddqFull.segment<6>(13);

	xDotEig.segment(nq_,ndq_) = ddq;

	// cout << "t: " << t << endl;

	// cout << "x:" << endl;
	// cout << xEig.transpose() << endl;

	// cout << "q:" << endl;
	// cout << q.transpose() << endl;

	// cout << "dq:" << endl;
	// cout << dq.transpose() << endl;

	// cout << "u:" << endl;
	// cout << u.transpose() << endl;

	// cout << "uWithFF:" << endl;
	// cout << uWithFF.transpose() << endl;

	// cout << "uFull:" << endl;
	// cout << uFull.transpose() << endl;

	// cout << "qFull:" << endl;
	// cout << qFull.transpose() << endl;

	// cout << "dqFull:" << endl;
	// cout << dqFull.transpose() << endl;

	// cout << "ddqFull:" << endl;
	// cout << ddqFull.transpose() << endl;

	// cout << "ddq:" << endl;
	// cout << ddq.transpose() << endl;

	// cout << "xDotEig:" << endl;
	// cout << xDotEig.transpose() << endl << endl;
}

void ExoSimulator::internalDynamics(const Eigen::VectorXd &q,
                                    const Eigen::VectorXd &dq,
                                          Eigen::VectorXd &u)
{
	// Joint friction
	for(uint32_t i = 0; i<nu_; i++)
	{
		u(i+6) = -options_.frictionViscous(i)*dq(i+6) - options_.frictionDry(i)*saturateSoft(dq(i+6)/options_.dryFictionVelEps,-1.0,1.0,0.7) ;
	}

	// u(0) = 10.0;
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

void ExoSimulator::checkCtrl(void)
{
	Eigen::VectorXd u = Eigen::VectorXd::Zero(nu_);
	Eigen::VectorXd x = Eigen::VectorXd::Zero(nx_);

	controller_(0.0,x,u);

	if(u.rows()!=nu_)
	{
		cout << "Error - ExoSimulator::checkCtrl - Controller returned input with wrong size " << u.rows() <<" instead of " << nu_ << "." << endl;
		tesc_ = false;
	}
}