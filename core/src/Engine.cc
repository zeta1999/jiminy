#include <iostream>
#include <algorithm>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/aba.hpp"

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
    nq_(),
    nv_(),
    nx_()
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

        model_->pncData_ = pinocchio::Data(model_->pncModel_); // Initialize the internal state
        for(auto it = itBegin; it != itEnd; it++)
        {
            float64_t const & t = it->second;
            state_t const & xVect = it->first;

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
        xVectDot.resize(nx_);

        Eigen::Map<const vectorN_t> xEig(xVect.data(),nx_);
        Eigen::Map<vectorN_t> xDotEig(xVectDot.data(),nx_);
        Eigen::Map<const vectorN_t> q(xVect.data(),nq_);
        Eigen::Map<const vectorN_t> dq(xVect.data() + nq_,nv_);
        Eigen::Map<vectorN_t> ddq(xVectDot.data() + nq_,nv_);

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

        // Compute kinematics information
        pinocchio::forwardKinematics(model_->pncModel_, model_->pncData_, qPinocchio, dqPinocchio);
        pinocchio::framesForwardKinematics(model_->pncModel_, model_->pncData_);

        // Compute the external forces
        pinocchio::container::aligned_vector<pinocchio::Force> fext(model_->pncModel_.joints.size(), pinocchio::Force::Zero());
        for(int32_t const & contactFrameIdx : model_->getContactFramesIdx())
        {
            pinocchio::Force const fFrame(contactDynamics(contactFrameIdx));
            int32_t const parentIdx = model_->pncModel_.frames[contactFrameIdx].parent;
            fext[parentIdx] += fFrame;
        }

        // Compute command and internal dynamics
        vectorN_t uCommand;
        controller_->compute_efforts(*this, t, qPinocchio, dqPinocchio, uCommand);
        vectorN_t uInternal;
        internalDynamics(t, qPinocchio, dqPinocchio, uInternal);

        // Compute dynamics
        ddq = pinocchio::aba(model_->pncModel_, model_->pncData_, qPinocchio, dqPinocchio, uCommand + uInternal, fext);

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
        xDotEig.segment(7,nq_-7) = dq.tail(nv_-6);
        xDotEig.segment(nq_,nv_) = ddq;

        // // Make sure that the output has the right shape
        // xVectDot.resize(nx_);

        // // Map std::vector input data to Eigen::VectorXd
        // Eigen::Map<vectorN_t const> x(xVect.data(),nx_);
        // Eigen::Map<vectorN_t const> q(x.data(),nq_);
        // Eigen::Map<vectorN_t const> v(x.data() + nq_, nv_);
        // Eigen::Map<vectorN_t> xDot(xVectDot.data(),nx_);

        // // Convert the body angular velocity in the right frame

        // // Compute true quaternion derivative
        // // It is not possible to use directly pinocchio::integrate method since it requires dt
        // // One way would be to save the time of the last successful iteration has a private property of the engine
        // Eigen::Vector3d const omega = v.segment<3>(3);
        // quaternion_t quat(q.segment<4>(3).data()); // Only way to initialize with [x,y,z,w] order
        // quat.normalize();
        // quaternion_t quatDot = quat * quaternion_t(0, 0.5 * omega(0), 0.5 * omega(1), 0.5 * omega(2));

        // // Compute kinematics information
        // pinocchio::forwardKinematics(model_->pncModel_, model_->pncData_, q, v);
        // pinocchio::framesForwardKinematics(model_->pncModel_, model_->pncData_);

        // // Compute the external forces
        // pinocchio::container::aligned_vector<pinocchio::Force> fext(model_->pncModel_.joints.size(), 
        //                                                             pinocchio::Force::Zero());
        // for(int32_t const & contactFrameIdx : model_->getContactFramesIdx())
        // {
        //     pinocchio::Force const fFrame(contactDynamics(contactFrameIdx));
        //     int32_t const parentIdx = model_->pncModel_.frames[contactFrameIdx].parent;
        //     fext[parentIdx] += fFrame;
        // }

        // // Compute command and internal dynamics
        // vectorN_t u = vectorN_t::Zero(nv_);
        // controller_->compute_efforts(*this, t, q, v, u);

        // // Compute dynamics
        // vectorN_t a = pinocchio::aba(model_->pncModel_, model_->pncData_, q, v, u, fext);

        // // Fill up xDot
        // xDot.head<3>() = v.head<3>();
        // xDot.segment<4>(3) = quatDot.coeffs(); // coeffs returns a vector [x,y,z,w]
        // xDot.segment(7, nq_ - 7) = v.tail(nv_ - 6);
        // xDot.segment(nq_, nv_) = a;
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

            // Compute friction force
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

            float64_t blendingFactor = std::min(qJointError / engineJointOptions_.boundTransitionEps, 1.0);
            forceJoint *= blendingFactor;

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