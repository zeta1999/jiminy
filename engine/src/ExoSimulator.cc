#include <map>
#include <iostream>
#include <iomanip>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"

#include "exo_simu/engine/ExoSimulator.h"
#include "exo_simu/engine/ExoSimulatorUtils.h"

namespace exo_simu
{
    ExoSimulator::ExoSimulator(void):
    log(),
    isInitialized_(false),
    urdfPath_(),
    controller_(),
    mdlOptionsHolder_(),
    simOptionsHolder_(),
    mdlOptions_(nullptr),
    simOptions_(nullptr),
    model_(),
    data_(model_),
    nq_(19),
    ndq_(18),
    nx_(nq_ + ndq_),
    nu_(12),
    nqFull_(21),
    ndqFull_(20),
    nxFull_(nq_ + ndq_),
    nuFull_(ndqFull_),
    tesc_(true),
    contactFramesNames_{std::string("LeftExternalToe"),
                        std::string("LeftInternalToe"),
                        std::string("LeftExternalHeel"),
                        std::string("LeftInternalHeel"),
                        std::string("RightInternalToe"),
                        std::string("RightExternalToe"),
                        std::string("RightInternalHeel"),
                        std::string("RightExternalHeel")},
    imuFramesNames_{std::string("PelvisIMU"),
                    std::string("ThoraxIMU"),
                    std::string("RightTibiaIMU"),
                    std::string("LeftTibiaIMU")},
    jointsNames_{std::string("LeftFrontalHipJoint"),
                 std::string("LeftTransverseHipJoint"),
                 std::string("LeftSagittalHipJoint"),
                 std::string("LeftSagittalKneeJoint"),
                 std::string("LeftSagittalAnkleJoint"),
                 std::string("LeftHenkeAnkleJoint"),
                 std::string("RightFrontalHipJoint"),
                 std::string("RightTransverseHipJoint"),
                 std::string("RightSagittalHipJoint"),
                 std::string("RightSagittalKneeJoint"),
                 std::string("RightSagittalAnkleJoint"),
                 std::string("RightHenkeAnkleJoint")},
    contactFramesIdx_(),
    imuFramesIdx_(),
    jointsIdx_()
    {
        setModelOptions(getDefaultModelOptions());
        setSimulationOptions(getDefaultSimulationOptions());
    }

    ExoSimulator::ExoSimulator(const std::string urdfPath):
    ExoSimulator()
    {
        init(urdfPath);
    }

    ExoSimulator::ExoSimulator(const std::string urdfPath,
                               const ConfigHolder & mdlOptions):
    ExoSimulator()
    {
        init(urdfPath, mdlOptions);
    }

    ExoSimulator::ExoSimulator(const std::string urdfPath,
                               const ConfigHolder & mdlOptions,
                               const ConfigHolder & simOptions):
    ExoSimulator()
    {
        init(urdfPath, mdlOptions, simOptions);
    }

    ExoSimulator::~ExoSimulator(void)
    {
    }


    void ExoSimulator::init(const std::string urdfPath)
    {
        setUrdfPath(urdfPath);
    }

    void ExoSimulator::init(const std::string urdfPath,
                            const ConfigHolder & mdlOptions)
    {
        setUrdfPath(urdfPath);
        setModelOptions(mdlOptions);
    }

    void ExoSimulator::init(const std::string urdfPath,
                            const ConfigHolder & mdlOptions,
                            const ConfigHolder & simuOptions)
    {
        setUrdfPath(urdfPath);
        setModelOptions(mdlOptions);
        setSimulationOptions(simuOptions);
    }


    ExoSimulator::result_t ExoSimulator::simulate(const vectorN_t & x0,
                                                  const float64_t & t0,
                                                  const float64_t & tf,
                                                  const float64_t & dt,
                                                  controller_t controller)
    {
        auto callbackFct = [](const float64_t t, const vectorN_t & x)->bool{ return true; };
        return simulate(x0,t0,tf,dt,controller,callbackFct);
    }

    ExoSimulator::result_t ExoSimulator::simulate(const vectorN_t & x0,
                                                  const float64_t & t0,
                                                  const float64_t & tf,
                                                  const float64_t & dt,
                                                  controller_t controller,
                                                  callbackFct_t callbackFct)
    {
        if(!tesc_)
        {
            std::cout << "Error - ExoSimulator::simulate - Initialization failed, will not run simulation" << std::endl;
            return ExoSimulator::result_t::ERROR_INIT_FAILED;        
        }

        if(x0.rows() != nx_)
        {
            std::cout << "Error - ExoSimulator::simulate - Size of x0 (" << x0.size() << ") inconsistent with model size (" << nx_ << ")." << std::endl;
            return ExoSimulator::result_t::ERROR_BAD_INPUT;
        }
        state_t xx0(x0.data(),x0.data() + x0.size());

        if(tf <= t0)
        {
            std::cout << "Error - ExoSimulator::simulate - Final time (" << tf << ") is less that initial time (" << t0 << ")." << std::endl;
            return ExoSimulator::result_t::ERROR_BAD_INPUT;
        }
        uint64_t nPts = round((tf - t0) / dt + 1.0);

        if(!checkCtrl(controller))
        {
            std::cout << "Error - ExoSimulator::checkCtrl - Controller returned input with wrong size." << std::endl;
            return ExoSimulator::result_t::ERROR_BAD_INPUT;        
        }
        controller_ = controller;

        if(nPts < 2)
        {
            std::cout << "Error - ExoSimulator::simulate - Number of integration points is less than 2." << std::endl;
            return ExoSimulator::result_t::ERROR_GENERIC;
        }
        log.resize(0);
        log.reserve(nPts); // Reserve the estimated average number of pts required.

        auto rhsBind = bind(&ExoSimulator::dynamicsCL, this,
                            std::placeholders::_1,
                            std::placeholders::_2,
                            std::placeholders::_3);
        auto stepper = make_controlled(simOptions_->tolAbs, simOptions_->tolRel, stepper_t()); // make_dense_output, make_controlled
        auto itBegin = make_adaptive_time_iterator_begin(stepper, rhsBind, xx0, t0, tf, dt); // const_step_time, adaptive_time
        auto itEnd = make_adaptive_time_iterator_end(stepper, rhsBind, xx0);

        data_ = pinocchio::Data(model_); // Initialize the internal state
        for(auto it = itBegin; it != itEnd; it++)
        {
            const state_t & x = it->first;
            const float64_t & t = it->second;
            state_t log_data(nx_ + 1);
            
            log_data[0] = t;
            copy(x.begin(), x.end(), log_data.begin() + 1);
            log.push_back(log_data);
            
            Eigen::Map<const vectorN_t> xEig(x.data(), nx_);
            if(!callbackFct(t,xEig))
            {
                break;
            }
        }

        log.shrink_to_fit();

        // Log post processing
        if(simOptions_->logController || simOptions_->logOptoforces || simOptions_->logIMUs)
        {
            for(uint32_t i = 0; i < log.size(); i++)
            {
                // Prepare some variables
                Eigen::Map<const vectorN_t> q(log[i].data() + 1,nq_);
                Eigen::Map<const vectorN_t> dq(log[i].data() + 1 + nq_,ndq_);
                Eigen::Vector4d quatVec = q.segment<4>(3);
                quatVec.normalize();
                vectorN_t qPinocchio = q;
                qPinocchio.segment<3>(3) = quatVec.tail<3>();
                qPinocchio(6) = quatVec(0);
                vectorN_t qFull(nqFull_);
                vectorN_t dqFull(ndqFull_);
                qFull.head<13>() = qPinocchio.head<13>();
                qFull.segment<6>(14) = qPinocchio.tail<6>();
                qFull(13) = 0.0;
                qFull(20) = 0.0;
                dqFull.head<12>() = dq.head<12>();
                dqFull.segment<6>(13) = dq.tail<6>();
                dqFull(12) = 0.0;
                dqFull(19) = 0.0;
                pinocchio::forwardKinematics(model_, data_, qFull, dqFull);
                pinocchio::framesForwardKinematics(model_, data_);

                // Compute foot contact forces
                Eigen::Matrix<float64_t,3,8> optoforces;
                if(simOptions_->logController || simOptions_->logOptoforces)
                {
                    for(uint32_t i = 0; i<contactFramesIdx_.size(); i++)
                    {
                        const pinocchio::Force fFrame(contactDynamics(contactFramesIdx_[i]));
                        optoforces.block<3,1>(0,i) = fFrame.linear();
                    }                
                }

                // Get IMUs data
                Eigen::Matrix<float64_t,7,4> IMUs;
                if(simOptions_->logController || simOptions_->logIMUs)
                {
                    for(uint32_t i = 0; i<imuFramesIdx_.size(); i++)
                    {
                        const Eigen::Matrix4d tformIMU = data_.oMf[imuFramesIdx_[i]].toHomogeneousMatrix();
                        const Eigen::Matrix3d rotIMU = tformIMU.topLeftCorner<3,3>();
                        const Eigen::Quaterniond quatIMU(rotIMU);
                        pinocchio::Motion motionIMU = pinocchio::getFrameVelocity(model_,data_,imuFramesIdx_[i]);
                        Eigen::Vector3d omegaIMU = motionIMU.angular();    
                        IMUs(0,i) = quatIMU.w();
                        IMUs(1,i) = quatIMU.x();
                        IMUs(2,i) = quatIMU.y();
                        IMUs(3,i) = quatIMU.z();
                        IMUs.block<3,1>(4,i) = omegaIMU;
                    }                
                }

                // Compute control action
                vectorN_t u = vectorN_t::Zero(nu_);
                if(simOptions_->logController)
                {
                    Eigen::Map<const vectorN_t> xEig(log[i].data() + 1,nx_);
                    const float64_t & t = log[i][0];
                    controller_(t,xEig,optoforces,IMUs,u);        
                }

                // Fill up log
                if(simOptions_->logOptoforces)
                {
                    log[i].insert(log[i].end(),optoforces.data(),optoforces.data() + 24);
                }
                if(simOptions_->logIMUs)
                {
                    log[i].insert(log[i].end(),IMUs.data(),IMUs.data() + 28);
                }
                if(simOptions_->logController)
                {
                    log[i].insert(log[i].end(),u.data(),u.data() + nu_);        
                }
            }
        }
        return ExoSimulator::result_t::SUCCESS;
    }

    std::string ExoSimulator::getUrdfPath(void)
    {
        return urdfPath_;
    }

    void ExoSimulator::setUrdfPath(const std::string &urdfPath)
    {
        urdfPath_ = urdfPath;
        pinocchio::urdf::buildModel(urdfPath,pinocchio::JointModelFreeFlyer(),model_);

        if(model_.nq != nqFull_ || model_.nv != ndqFull_)
        {
            std::cout << "Error - ExoSimulator::setUrdfPath - Urdf not recognized." << std::endl;
            tesc_ = false;
        }

        // Find contact frames idx
        for(uint32_t i = 0; i < contactFramesNames_.size(); i++)
        {
            if(!model_.existFrame(contactFramesNames_[i]))
            {
                std::cout << "Error - ExoSimulator::setUrdfPath - Frame '" << contactFramesNames_[i] << "' not found in urdf." << std::endl;
                tesc_ = false;    
                return;        
            }
            contactFramesIdx_.push_back(model_.getFrameId(contactFramesNames_[i]));
        }

        // Find IMU frames idx
        for(uint32_t i = 0; i < imuFramesNames_.size(); i++)
        {
            if(!model_.existFrame(imuFramesNames_[i]))
            {
                std::cout << "Error - ExoSimulator::setUrdfPath - Frame '" << imuFramesNames_[i] << "' not found in urdf." << std::endl;
                tesc_ = false;    
                return;        
            }
            imuFramesIdx_.push_back(model_.getFrameId(imuFramesNames_[i]));
        }

        // Find joints idx
        for(uint32_t i = 0; i < jointsNames_.size(); i++)
        {
            if(!model_.existJointName(jointsNames_[i]))
            {
                std::cout << "Error - ExoSimulator::setUrdfPath - Joint '" << jointsNames_[i] << "' not found in urdf." << std::endl;
                tesc_ = false;    
                return;        
            }
            jointsIdx_.push_back(model_.getJointId(jointsNames_[i]));
        }

        isInitialized_ = true;
        
        setModelOptions(getModelOptions()); // Make sure bounds are updated if necessary
    }

    ConfigHolder ExoSimulator::getModelOptions(void)
    {
        return mdlOptionsHolder_;
    }

    void ExoSimulator::setModelOptions(const ConfigHolder & mdlOptions)
    {
        mdlOptionsHolder_ = mdlOptions;
        model_.gravity = mdlOptions.get<vectorN_t>("gravity");

        ConfigHolder& jointOptionsHolder_ = mdlOptionsHolder_.get<ConfigHolder>("joints");
        if(isInitialized_ && jointOptionsHolder_.get<bool>("boundsFromUrdf"))
        {
            jointOptionsHolder_.get<vectorN_t>("boundsMin").head<6>() = model_.lowerPositionLimit.segment<6>(7);
            jointOptionsHolder_.get<vectorN_t>("boundsMin").tail<6>() = model_.lowerPositionLimit.segment<6>(14);
            jointOptionsHolder_.get<vectorN_t>("boundsMax").head<6>() = model_.upperPositionLimit.segment<6>(7);
            jointOptionsHolder_.get<vectorN_t>("boundsMax").tail<6>() = model_.upperPositionLimit.segment<6>(14);
        }

        mdlOptions_ = std::shared_ptr<modelOptions_t>(new modelOptions_t(mdlOptionsHolder_));
    }

    ConfigHolder ExoSimulator::getSimulationOptions(void)
    {
        return simOptionsHolder_;
    }

    void ExoSimulator::setSimulationOptions(const ConfigHolder & simOptions)
    {
        simOptionsHolder_ = simOptions;
        simOptions_ = std::shared_ptr<simulationOptions_t>(new simulationOptions_t(simOptionsHolder_));
    }

    void ExoSimulator::dynamicsCL(const state_t &x,
                                        state_t &xDot,
                                  const float64_t t)
    {
        xDot.resize(nx_);

        Eigen::Map<const vectorN_t> xEig(x.data(),nx_);
        Eigen::Map<vectorN_t> xDotEig(xDot.data(),nx_);
        Eigen::Map<const vectorN_t> q(x.data(),nq_);
        Eigen::Map<const vectorN_t> dq(x.data() + nq_,ndq_);
        Eigen::Map<vectorN_t> ddq(xDot.data() + nq_,ndq_);
        vectorN_t u = vectorN_t::Zero(nu_);
        vectorN_t uWithFF = vectorN_t::Zero(ndq_);
        vectorN_t uInternal = vectorN_t::Zero(ndq_);

        // Compute quaternion derivative
        const Eigen::Vector3d omega = dq.segment<3>(3);
        Eigen::Vector4d quatVec = q.segment<4>(3);
        quatVec.normalize();
        const Eigen::Quaterniond quat(quatVec(0),quatVec(1),quatVec(2),quatVec(3));
        Eigen::Matrix<float64_t,4,4> quat2quatDot;
        quat2quatDot << 0        , omega(0), omega(1), omega(2),
                        -omega(0),        0,-omega(2), omega(1),
                        -omega(1), omega(2),        0,-omega(0),
                        -omega(2), -omega(1), omega(0),       0;
        quat2quatDot *= -0.5;
        const Eigen::Vector4d quatDot = quat2quatDot*quatVec;

        // Put quaternion in [x y z w] order
        vectorN_t qPinocchio = q;
        qPinocchio.segment<3>(3) = quatVec.tail<3>();
        qPinocchio(6) = quatVec(0);

        // Compute body velocity
        vectorN_t dqPinocchio = dq;
        const Eigen::Matrix3d Rb2w = quat.toRotationMatrix();
        dqPinocchio.head<3>() = Rb2w.transpose()*dqPinocchio.head<3>();

        // Stuff Specific to wandercraft urdf
        vectorN_t qFull(nqFull_);
        vectorN_t dqFull(ndqFull_);
        vectorN_t ddqFull(ndqFull_);
        vectorN_t uFull = vectorN_t::Zero(nuFull_);

        qFull.head<13>() = qPinocchio.head<13>();
        qFull.segment<6>(14) = qPinocchio.tail<6>();
        qFull(13) = 0.0;
        qFull(20) = 0.0;

        dqFull.head<12>() = dqPinocchio.head<12>();
        dqFull.segment<6>(13) = dqPinocchio.tail<6>();
        dqFull(12) = 0.0;
        dqFull(19) = 0.0;

        // Compute foot contact forces
        pinocchio::forwardKinematics(model_, data_, qFull, dqFull);
        pinocchio::framesForwardKinematics(model_, data_);
        Eigen::Matrix<float64_t,3,8> optoforces;
        pinocchio::container::aligned_vector<pinocchio::Force> fext(model_.joints.size(),pinocchio::Force::Zero());
        for(uint32_t i = 0; i<contactFramesIdx_.size(); i++)
        {
            const int32_t parentIdx = model_.frames[contactFramesIdx_[i]].parent;
            const pinocchio::Force fFrame(contactDynamics(contactFramesIdx_[i]));
            optoforces.block<3,1>(0,i) = fFrame.linear();
            fext[parentIdx] += fFrame;
        }

        // Get IMUs data
        Eigen::Matrix<float64_t,7,4> IMUs;
        for(uint32_t i = 0; i < imuFramesIdx_.size(); i++)
        {
            const Eigen::Matrix4d tformIMU = data_.oMf[imuFramesIdx_[i]].toHomogeneousMatrix();
            const Eigen::Matrix3d rotIMU = tformIMU.topLeftCorner<3,3>();
            const Eigen::Quaterniond quatIMU(rotIMU);
            pinocchio::Motion motionIMU = pinocchio::getFrameVelocity(model_,data_,imuFramesIdx_[i]);
            Eigen::Vector3d omegaIMU = motionIMU.angular();    
            IMUs(0,i) = quatIMU.w();
            IMUs(1,i) = quatIMU.x();
            IMUs(2,i) = quatIMU.y();
            IMUs(3,i) = quatIMU.z();
            IMUs.block<3,1>(4,i) = omegaIMU;
        }

        // Compute control input
        controller_(t,xEig,optoforces,IMUs,u);
        uWithFF.tail(ndq_ - 6) = u;

        // Compute internal dynamics
        internalDynamics(q,dq,uInternal);
        uWithFF += uInternal;

        // Stuf Specific to wandercraft urdf
        uFull.head<12>() = uWithFF.head<12>();
        uFull.segment<6>(13) = uWithFF.tail<6>();
        uFull(12) = 0.0;
        uFull(19) = 0.0;

        // Compute dynamics
        ddqFull = pinocchio::aba(model_, data_, qFull, dqFull, uFull, fext);

        // Stuf Specific to wandercraft urdf
        ddq.head<12>() = ddqFull.head<12>();
        ddq.tail<6>() = ddqFull.segment<6>(13);

        // Compute world frame acceleration
        Eigen::Matrix3d Rw2bDot;
        Rw2bDot(0,0) = -4*quatVec(2)*quatDot(2) - 4*quatVec(3)*quatDot(3);
        Rw2bDot(0,1) =  2*quatDot(1)*quatVec(2) + 2*quatVec(1)*quatDot(2) + 2*quatDot(0)*quatVec(3) + 2*quatVec(0)*quatDot(3);
        Rw2bDot(0,2) = -2*quatDot(0)*quatVec(2) - 2*quatVec(0)*quatDot(2) + 2*quatDot(1)*quatVec(3) + 2*quatVec(1)*quatDot(3);
        Rw2bDot(1,0) =  2*quatDot(1)*quatVec(2) + 2*quatVec(1)*quatDot(2) - 2*quatDot(0)*quatVec(3) - 2*quatVec(0)*quatDot(3);
        Rw2bDot(1,1) = -4*quatVec(1)*quatDot(1) - 4*quatVec(3)*quatDot(3);
        Rw2bDot(1,2) =  2*quatDot(0)*quatVec(1) + 2*quatVec(0)*quatDot(1) + 2*quatDot(2)*quatVec(3) + 2*quatVec(2)*quatDot(3);
        Rw2bDot(2,0) =  2*quatDot(0)*quatVec(2) + 2*quatVec(0)*quatDot(2) + 2*quatDot(1)*quatVec(3) + 2*quatVec(1)*quatDot(3);
        Rw2bDot(2,1) = -2*quatDot(0)*quatVec(1) - 2*quatVec(0)*quatDot(1) + 2*quatDot(2)*quatVec(3) + 2*quatVec(2)*quatDot(3);
        Rw2bDot(2,2) = -4*quatVec(1)*quatDot(1) - 4*quatVec(2)*quatDot(2);
        ddq.head<3>() = Rb2w*(ddq.head<3>() - Rw2bDot*dq.head<3>());

        // Fill up xDot
        xDotEig.head<3>() = dq.head<3>();
        xDotEig.segment<4>(3) = quatDot;
        xDotEig.segment(7,nq_-7) = dq.tail(ndq_-6);
        xDotEig.segment(nq_,ndq_) = ddq;
    }

    void ExoSimulator::internalDynamics(const vectorN_t &q,
                                        const vectorN_t &dq,
                                            vectorN_t &u)
    {
        const jointOptions_t * const jointOptions_ = &mdlOptions_->joints;

        // Joint friction
        for(uint32_t i = 0; i<nu_; i++)
        {
            u(i+6) = -jointOptions_->frictionViscous(i)*dq(i+6) - jointOptions_->frictionDry(i) * \
                     saturateSoft(dq(i+6) / jointOptions_->dryFrictionVelEps,-1.0,1.0,0.7);
        }

        // Joint bounds
        for(uint32_t i = 0; i<nu_; i++)
        {
            const float64_t qJt = q(i+7);
            const float64_t dqJt = dq(i+6);
            const float64_t qJtMin = jointOptions_->boundsMin(i);
            const float64_t qJtMax = jointOptions_->boundsMax(i);

            float64_t fJt;
            if(qJt > qJtMax)
            {
                const float64_t qErr = qJt - qJtMax;
                float64_t damping = 0;
                if(dqJt > 0)
                {
                    damping = -jointOptions_->boundDamping*dqJt;
                }

                fJt = -jointOptions_->boundStiffness * qErr + damping;

                float64_t blendingFactor = qErr / jointOptions_->boundTransitionEps;
                if(blendingFactor>1.0)
                {
                    blendingFactor = 1.0;
                }

                fJt *= blendingFactor;
                u(i + 6) += fJt;
            }

            if(qJt < qJtMin)
            {
                const float64_t qErr = qJtMin - qJt;
                float64_t damping = 0;
                if(dqJt < 0)
                {
                    damping = -jointOptions_->boundDamping * dqJt;
                }

                fJt = jointOptions_->boundStiffness * qErr + damping;

                float64_t blendingFactor = qErr / jointOptions_->boundTransitionEps;
                if(blendingFactor>1.0)
                {
                    blendingFactor = 1.0;
                }

                fJt *= blendingFactor;
                u(i + 6) += fJt;
            }
        }
    }

    vectorN_t ExoSimulator::contactDynamics(const int32_t & frameId)
    {
        const contactOptions_t * const contactOptions_ = &mdlOptions_->contacts;
        
        Eigen::Matrix4d tformFrame = data_.oMf[frameId].toHomogeneousMatrix();
        Eigen::Vector3d posFrame = tformFrame.topRightCorner<3,1>();

        vectorN_t fextLocal = vectorN_t::Zero(6);

        if(posFrame(2) < 0.0)
        {
            // Get various transformations
            const Eigen::Matrix4d tformFrame2Jt = model_.frames[frameId].placement.toHomogeneousMatrix();
            Eigen::Vector3d fextInWorld(0.0,0.0,0.0);
            const Eigen::Vector3d posFrameJoint = tformFrame2Jt.topRightCorner<3,1>();
            const pinocchio::Motion motionFrame = pinocchio::getFrameVelocity(model_,data_,frameId);
            const Eigen::Vector3d vFrameInWorld = tformFrame.topLeftCorner<3,3>()*motionFrame.linear();

            // Compute normal force
            float64_t damping = 0;
            if(vFrameInWorld(2)<0)
            {
                damping = -contactOptions_->damping * vFrameInWorld(2);
            }
            fextInWorld(2) = -contactOptions_->stiffness * posFrame(2) + damping;

            // Compute friction force
            const Eigen::Vector2d vxy = vFrameInWorld.head<2>();
            const float64_t vNorm = vxy.norm();
            float64_t frictionCoeff;
            if(vNorm > contactOptions_->dryFrictionVelEps)
            {
                if(vNorm < 1.5 * contactOptions_->dryFrictionVelEps)
                {
                    frictionCoeff = -2.0 * vNorm * (contactOptions_->frictionDry - contactOptions_->frictionViscous) \
                        / contactOptions_->dryFrictionVelEps + 3.0*contactOptions_->frictionDry - \
                        2.0*contactOptions_->frictionViscous;
                }
                else
                {
                    frictionCoeff = contactOptions_->frictionViscous;
                }
            }
            else
            {
                frictionCoeff = vNorm * contactOptions_->frictionDry / contactOptions_->dryFrictionVelEps;
            }
            fextInWorld.head<2>() = -vxy * frictionCoeff * fextInWorld(2);

            // Express forces at parent joint frame origin
            fextLocal.head<3>() = tformFrame2Jt.topLeftCorner<3,3>()*tformFrame.topLeftCorner<3,3>().transpose()*fextInWorld;
            fextLocal.tail<3>() = posFrameJoint.cross(fextLocal.head<3>()).eval();

            // Add blending factor
            float64_t blendingFactor = -posFrame(2) / contactOptions_->transitionEps;
            if(blendingFactor > 1.0)
            {
                blendingFactor = 1.0;
            }
            fextLocal *= blendingFactor;
        }

        return fextLocal;
    }

    float64_t ExoSimulator::saturateSoft(const float64_t in,
                                         const float64_t mi,
                                         const float64_t ma,
                                         const float64_t r)
    {
        float64_t uc, range, middle, bevelL, bevelXc, bevelYc, bevelStart, bevelStop, out;
        const float64_t alpha = M_PI/8;
        const float64_t beta = M_PI/4;

        range = ma - mi;
        middle = (ma + mi)/2;
        uc = 2*(in - middle)/range;

        bevelL = r * tan(alpha);
        bevelStart = 1 - cos(beta)*bevelL;
        bevelStop = 1 + bevelL;
        bevelXc = bevelStop;
        bevelYc = 1 - r;

        if(uc >= bevelStop)
        {
            out = ma;
        }
        else if(uc <= -bevelStop)
        {
            out = mi;
        }
        else if(uc <= bevelStart && uc >= -bevelStart)
        {
            out = in;
        }
        else if(uc > bevelStart)
        {
            out = sqrt(r * r - (uc - bevelXc) * (uc - bevelXc)) + bevelYc;
            out = 0.5 * out * range + middle;
        }
        else if(uc<-bevelStart)
        {
            out = -sqrt(r * r - (uc + bevelXc) * (uc + bevelXc)) - bevelYc;
            out = 0.5 * out * range + middle;
        }
        else
        {
            out = in;            
        }
        return out;
    }

    bool ExoSimulator::checkCtrl(controller_t controller)
    {
        vectorN_t u = vectorN_t::Zero(nu_);
        vectorN_t x = vectorN_t::Zero(nx_);
        Eigen::Matrix<float64_t,3,8> optoforces = Eigen::Matrix<float64_t,3,8>::Zero();
        Eigen::Matrix<float64_t,7,4> IMUs = Eigen::Matrix<float64_t,7,4>::Zero();

        controller(0.0,x,optoforces,IMUs,u);

        if(u.rows() != nu_)
        {
            return false;
        }
        else
        {
            return true;
        }
    }
}