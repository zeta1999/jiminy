#include <iostream>
#include <cmath>
#include <algorithm>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include "exo_simu/core/Utilities.h"
#include "exo_simu/core/AbstractController.h"
#include "exo_simu/core/Engine.h"

namespace exo_simu
{
    Engine::Engine(void):
    log(),
    engineOptions_(nullptr),
    isInitialized_(false),
    model_(nullptr),
    controller_(nullptr),
    engineOptionsHolder_(),
    callbackFct_([](float64_t const & t, 
                    vectorN_t const & x)->bool{ return true; }),
    timePrev_(0),
    nq_(0),
    nv_(0),
    nx_(0)
    {
        setOptions(getDefaultOptions());
    }

    Engine::~Engine(void)
    {
        // Empty
    }

    result_t Engine::initialize(Model              & model,
                                AbstractController & controller,
                                callbackFct_t        callbackFct)
    {
        result_t returnCode = result_t::SUCCESS;

        if (!model.getIsInitialized())
        {
            std::cout << "Error - Engine::initialize - Model not initialized." << std::endl;
            return result_t::ERROR_INIT_FAILED;
        }
        model_ = std::shared_ptr<Model>(model.clone());
        nq_ = model_->pncModel_.nq;
        nv_ = model_->pncModel_.nv;
        nx_ = nq_ + nv_;

        if (!controller.getIsInitialized())
        {
            std::cout << "Error - Engine::initialize - Controller not initialized." << std::endl;
            return result_t::ERROR_INIT_FAILED;
        }
        if (returnCode == result_t::SUCCESS)
        {
            try
            {
                float64_t t = 0;
                vectorN_t q = vectorN_t::Zero(nq_);
                vectorN_t v = vectorN_t::Zero(nv_);
                vectorN_t u = vectorN_t::Zero(nv_);
                returnCode = controller.compute_efforts(*this, t, q, v, u);
                if(u.size() != nv_)
                {
                    std::cout << "Error - Engine::initialize - The controller returns command with wrong size." << std::endl;
                    returnCode = result_t::ERROR_BAD_INPUT;
                }
            }
            catch (std::exception& e)
            {
                std::cout << "Error - Engine::initialize - Something is wrong with the Controller. Impossible to compute command." << std::endl;
                returnCode = result_t::ERROR_GENERIC;
            }
        }
        if (returnCode == result_t::SUCCESS)
        {
            controller_ = std::shared_ptr<AbstractController>(controller.clone());
        }

        if (returnCode == result_t::SUCCESS)
        {
            // TODO: Check that the callback function is working as expected
            callbackFct_ = callbackFct;
        }

        if (returnCode == result_t::SUCCESS)
        {
            isInitialized_ = true;
            setOptions(engineOptionsHolder_); // Make sure the gravity is properly set at model level
        }

        return returnCode;
    }

    result_t Engine::simulate(vectorN_t const & x0,
                              float64_t const & tf,
                              float64_t const & dt)
    {
        result_t returnCode = result_t::SUCCESS;

        if(!isInitialized_)
        {
            std::cout << "Error - Engine::simulate - Engine not initialized. Impossible to run the simulation." << std::endl;
            return result_t::ERROR_INIT_FAILED;
        }

        if(x0.rows() != nx_)
        {
            std::cout << "Error - Engine::simulate - Size of x0 (" << x0.size() << ") inconsistent with model size (" << nx_ << ")." << std::endl;
            return result_t::ERROR_BAD_INPUT;
        }
        state_t x0Vect(x0.data(), x0.data() + x0.size());

        if(tf <= 0)
        {
            std::cout << "Error - Engine::simulate - Final time (" << tf << ") is must be larger than 0." << std::endl;
            return result_t::ERROR_BAD_INPUT;
        }
        uint64_t nPts = ceil(tf / dt) + 1;

        if(nPts < 2)
        {
            std::cout << "Error - Engine::simulate - Number of integration points is less than 2." << std::endl;
            return result_t::ERROR_GENERIC;
        }

        log.resize(0);
        log.reserve(nPts); // Reserve the estimated average number of pts required.

        auto rhsBind = bind(&Engine::dynamicsCL, this,
                            std::placeholders::_3,
                            std::placeholders::_1,
                            std::placeholders::_2);
        auto stepper = make_controlled(engineOptions_->stepper.tolAbs, 
                                       engineOptions_->stepper.tolRel,
                                       stepper_t()); // make_dense_output, make_controlled
        auto itBegin = make_adaptive_time_iterator_begin(stepper, rhsBind, x0Vect, 0, tf, dt); // const_step_time, adaptive_time
        auto itEnd = make_adaptive_time_iterator_end(stepper, rhsBind, x0Vect);

        timePrev_ = 0;
        model_->pncData_ = pinocchio::Data(model_->pncModel_); // Initialize the internal state
        for(auto it = itBegin; it != itEnd; it++)
        {
            float64_t const & t = it->second;
            state_t const & xVect = it->first;
            timePrev_ = t;

            state_t log_data(nx_ + 1);
            log_data[0] = t;
            copy(xVect.begin(), xVect.end(), log_data.begin() + 1);
            log.push_back(log_data);

            Eigen::Map<vectorN_t const> x(xVect.data(), nx_);
            if(!callbackFct_(t, x))
            {
                break;
            }
        }

        log.shrink_to_fit();
        
        return returnCode;
    }

    void Engine::dynamicsCL(float64_t const & t,
                            state_t   const & xVect,
                            state_t         & xVectDot)
    {
        /* Note that the position of the free flyer is in world frame,
           whereas the velocities and accelerations are relative to the
           parent body frame. */

        // Make sure that the output has the right shape
        xVectDot.resize(nx_);

        // Map std::vector input data to Eigen::VectorXd
        Eigen::Map<vectorN_t const> q(xVect.data(),nq_);
        Eigen::Map<vectorN_t const> v(xVect.data() + nq_, nv_);
        Eigen::Map<vectorN_t> xDot(xVectDot.data(),nx_);

        // Compute kinematics information
        pinocchio::forwardKinematics(model_->pncModel_, model_->pncData_, q, v);
        pinocchio::framesForwardKinematics(model_->pncModel_, model_->pncData_);

        // Compute the external forces
        pinocchio::container::aligned_vector<pinocchio::Force> fext(model_->pncModel_.joints.size(), 
                                                                    pinocchio::Force::Zero());
        for(int32_t const & contactFrameIdx : model_->getContactFramesIdx())
        {
            pinocchio::Force const fFrame(contactDynamics(contactFrameIdx));
            int32_t parentIdx = model_->pncModel_.frames[contactFrameIdx].parent;
            fext[parentIdx] += fFrame;
        }

        // Compute command and internal dynamics
        vectorN_t uCommand = vectorN_t::Zero(nv_);
        controller_->compute_efforts(*this, t, q, v, uCommand);
        vectorN_t uInternal = vectorN_t::Zero(nv_);
        internalDynamics(t, q, v, uInternal);

        // Compute dynamics
        vectorN_t a = pinocchio::aba(model_->pncModel_, model_->pncData_, q, v, uCommand + uInternal, fext);

        /* Hack to compute the configuration vector derivative,
           including the quaternions on SO3 automatically. Note  
           that the time difference must not be too small to
           avoid failure. */
        float64_t dt = std::max(1e-5, t - timePrev_);
        vectorN_t qNext = vectorN_t::Zero(nq_);
        pinocchio::integrate(model_->pncModel_, q, v*dt, qNext);
        vectorN_t qDot = (qNext - q) / dt;

        // Fill up xDot
        xDot.head(nq_) = qDot;
        xDot.tail(nv_) = a;
    }
 
    vectorN_t Engine::contactDynamics(int32_t const & frameId) const
    {
        // /* /!\ Note that the contact dynamics depends only on kinematics data. /!\ */

        contactOptions_t const * const contactOptions_ = &engineOptions_->contacts;
        
        Eigen::Matrix4d tformFrame = model_->pncData_.oMf[frameId].toHomogeneousMatrix();
        Eigen::Vector3d posFrame = tformFrame.topRightCorner<3,1>();

        vectorN_t fextLocal = vectorN_t::Zero(6);

        if(posFrame(2) < 0.0)
        {
            // Get various transformations
            Eigen::Matrix4d const tformFrame2Jt = model_->pncModel_.frames[frameId].placement.toHomogeneousMatrix();
            Eigen::Vector3d fextInWorld(0.0,0.0,0.0);
            Eigen::Vector3d const posFrameJoint = tformFrame2Jt.topRightCorner<3,1>();
            pinocchio::Motion const motionFrame = pinocchio::getFrameVelocity(model_->pncModel_, 
                                                                              model_->pncData_,
                                                                              frameId);
            Eigen::Vector3d const vFrameInWorld = tformFrame.topLeftCorner<3,3>() * motionFrame.linear();

            // Compute normal force
            float64_t damping = 0;
            if(vFrameInWorld(2) < 0)
            {
                damping = -contactOptions_->damping * vFrameInWorld(2);
            }
            fextInWorld(2) = -contactOptions_->stiffness * posFrame(2) + damping;

            // Compute friction forces
            Eigen::Vector2d const vxy = vFrameInWorld.head<2>();
            float64_t const vNorm = vxy.norm();
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

            // Compute the forces at the origin of the parent joint frame
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

    void Engine::internalDynamics(float64_t const & t,
                                  vectorN_t const & q,
                                  vectorN_t const & v,
                                  vectorN_t       & u)
    {
        // Enforce the bounds of the actuated joints of the model
        u = vectorN_t::Zero(nv_);

        Model::jointOptions_t const & mdlJointOptions_ = model_->mdlOptions_->joints;
        Engine::jointOptions_t const & engineJointOptions_ = engineOptions_->joints;

        std::vector<int32_t> jointsPositionIdx = model_->getJointsPositionIdx();
        std::vector<int32_t> jointsVelocityIdx = model_->getJointsVelocityIdx();
        for (uint32_t i = 0; i<jointsPositionIdx.size(); i++)
        {
            float64_t const qJoint = q(jointsPositionIdx[i]);
            float64_t const vJoint = v(jointsVelocityIdx[i]);
            float64_t const qJointMin = mdlJointOptions_.boundsMin(i);
            float64_t const qJointMax = mdlJointOptions_.boundsMax(i);

            float64_t forceJoint = 0;
            float64_t qJointError = 0;
            if (qJoint > qJointMax)
            {
                qJointError = qJoint - qJointMax;
                float64_t damping = -engineJointOptions_.boundDamping * std::max(vJoint, 0.0);
                forceJoint = -engineJointOptions_.boundStiffness * qJointError + damping;
            }
            else if (qJoint < qJointMin)
            {
                qJointError = qJointMin - qJoint;
                float64_t damping = -engineJointOptions_.boundDamping * std::min(vJoint, 0.0);
                forceJoint = engineJointOptions_.boundStiffness * qJointError + damping;
            }

            float64_t blendingFactor = qJointError / engineJointOptions_.boundTransitionEps;
            float64_t blendingLaw = std::tanh(2 * blendingFactor);
            forceJoint *= blendingLaw;

            u(jointsVelocityIdx[i]) += forceJoint;
        }
    }

    configHolder_t Engine::getOptions(void) const
    {
        return engineOptionsHolder_;
    }

    void Engine::setOptions(configHolder_t const & engineOptions)
    {
        engineOptionsHolder_ = engineOptions;
        if (isInitialized_)
        {
            model_->pncModel_.gravity = boost::get<vectorN_t>(engineOptions.at("gravity")); // It is reversed (Third Newton law)
        }
        engineOptions_ = std::shared_ptr<engineOptions_t const>(new engineOptions_t(engineOptionsHolder_));
    }

    bool Engine::getIsInitialized(void) const
    {
        return isInitialized_;
    }

    Model const & Engine::getModel(void) const
    {
        return *model_.get();
    }

    uint32_t Engine::nq(void) const
    {
        return nq_;
    }

    uint32_t Engine::nv(void) const
    {
        return nv_;
    }

    uint32_t Engine::nx(void) const
    {
        return nx_;
    }
}