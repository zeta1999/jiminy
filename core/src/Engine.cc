#include <iostream>

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
    engineOptions_(nullptr),
    isInitialized_(false),
    model_(),
    controller_(nullptr),
    logger_(),
    engineOptionsHolder_()
    {
        setOptions(getDefaultOptions());
    }

    Engine::~Engine(void)
    {
        // Empty
    }

    result_t Engine::initialize(Model              & model,
                                AbstractController & controller)
    {
        result_t returnCode = result_t::SUCCESS;

        if (!model.getIsInitialized())
        {
            std::cout << "Error - Engine::initialize - Model not initialized." << std::endl;
            return result_t::ERROR_INIT_FAILED;
        }
        model_ = model;

        if (!controller.getIsInitialized())
        {
            std::cout << "Error - Engine::initialize - Controller not initialized." << std::endl;
            return result_t::ERROR_INIT_FAILED;
        }
        if (returnCode == result_t::SUCCESS)
        {
            float64_t t = 0;
            vectorN_t q = vectorN_t::Zero(model.pncModel_.nq);
            vectorN_t v = vectorN_t::Zero(model.pncModel_.nv);
            vectorN_t u = vectorN_t::Zero(model.pncModel_.nv);

            try
            {
                returnCode = controller.compute_efforts(*this, t, q, v, u);
                if(u.rows() != model.pncModel_.nv)
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
            returnCode = logger_.initialize(model_);
        }

        if (returnCode == result_t::SUCCESS)
        {
            isInitialized_ = true;
        }

        return returnCode;
    }

    result_t Engine::simulate(vectorN_t const & x0,
                              float64_t const & tf,
                              float64_t const & dt)
    {
        auto callbackFct = [](float64_t const t, vectorN_t const & x)->bool{ return true; };
        return simulate(x0,tf,dt,callbackFct);
    }

    result_t Engine::simulate(vectorN_t     const & x0,
                              float64_t     const & tf,
                              float64_t     const & dt,
                              callbackFct_t         callbackFct)
    {
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
        state_t xx0(x0.data(), x0.data() + x0.size());

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

        auto rhsBind = bind(&Engine::dynamicsCL, this,
                            std::placeholders::_1,
                            std::placeholders::_2,
                            std::placeholders::_3);
        auto stepper = make_controlled(simOptions_->tolAbs, simOptions_->tolRel, stepper_t()); // make_dense_output, make_controlled
        auto itBegin = make_adaptive_time_iterator_begin(stepper, rhsBind, xx0, 0, tf, dt); // const_step_time, adaptive_time
        auto itEnd = make_adaptive_time_iterator_end(stepper, rhsBind, xx0);

        logger.clear();
        model.pncData_ = pinocchio::Data(model.pncModel_); // Initialize the internal state
        for(auto it = itBegin; it != itEnd; it++)
        {
            logger.fetch();

            float64_t const & t = it->second;
            state_t const & x = it->first;
            Eigen::Map<vectorN_t const> xEigen(x.data(), nx_);
            if(!callbackFct(t,xEigen))
            {
                break;
            }
        }
        
        return result_t::SUCCESS;
    }

    configHolder_t Engine::getOptions(void) const
    {
        return engineOptionsHolder_;
    }

    void Engine::setOptions(configHolder_t const & engineOptions)
    {
        engineOptionsHolder_ = engineOptions;
        model_.pncModel_.gravity = boost::get<vectorN_t>(engineOptions.at("gravity"));
        engineOptions_ = std::shared_ptr<engineOptions_t const>(new engineOptions_t(engineOptionsHolder_));
    }

    bool Engine::getIsInitialized(void) const
    {
        return isInitialized_;
    }

    std::string Engine::getLog(bool const & isHeaderEnable, 
                               bool const & isDataEnable) const
    {
        return logger_.get(isHeaderEnable, isDataEnable);
    }

    Model const & Engine::getModel(void) const
    {
        return model_;
    }

    void Engine::dynamicsCL(state_t   const & x,
                            state_t         & xDot,
                            float64_t const   t)
    {
        xDot.resize(nx_);

        Eigen::Map<vectorN_t const> xEig(x.data(),nx_);
        Eigen::Map<vectorN_t> xDotEig(xDot.data(),nx_);
        Eigen::Map<vectorN_t const> q(x.data(),nq_);
        Eigen::Map<vectorN_t const> dq(x.data() + nq_,ndq_);
        Eigen::Map<vectorN_t> ddq(xDot.data() + nq_,ndq_);
        vectorN_t u = vectorN_t::Zero(nu_);
        vectorN_t uWithFF = vectorN_t::Zero(ndq_);
        vectorN_t uInternal = vectorN_t::Zero(ndq_);

        // Compute quaternion derivative
        Eigen::Vector3d const omega = dq.segment<3>(3);
        Eigen::Vector4d quatVec = q.segment<4>(3);
        quatVec.normalize();
        quaternion_t const quat(quatVec(0),quatVec(1),quatVec(2),quatVec(3));
        Eigen::Matrix<float64_t,4,4> quat2quatDot;
        quat2quatDot << 0        , omega(0), omega(1), omega(2),
                        -omega(0),        0,-omega(2), omega(1),
                        -omega(1), omega(2),        0,-omega(0),
                        -omega(2), -omega(1), omega(0),       0;
        quat2quatDot *= -0.5;
        Eigen::Vector4d const quatDot = quat2quatDot*quatVec;

        // Put quaternion in [x y z w] order
        vectorN_t qPinocchio = q;
        qPinocchio.segment<3>(3) = quatVec.tail<3>();
        qPinocchio(6) = quatVec(0);

        // Compute body velocity
        vectorN_t dqPinocchio = dq;
        Eigen::Matrix3d const Rb2w = quat.toRotationMatrix();
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
        pinocchio::forwardKinematics(model.pncModel_, model.pncData_, qFull, dqFull);
        pinocchio::framesForwardKinematics(model.pncModel_, model.pncData_);
        Eigen::Matrix<float64_t,3,8> optoforces;
        pinocchio::container::aligned_vector<pinocchio::Force> fext(model.pncModel_.joints.size(), 
                                                                    pinocchio::Force::Zero());
        for(uint32_t i = 0; i<contactFramesIdx_.size(); i++)
        {
            int32_t const parentIdx = model.pncModel_.frames[contactFramesIdx_[i]].parent;
            pinocchio::Force const fFrame(contactDynamics(contactFramesIdx_[i]));
            optoforces.block<3,1>(0,i) = fFrame.linear();
            fext[parentIdx] += fFrame;
        }

        // Get IMUs data
        Eigen::Matrix<float64_t,7,4> IMUs;
        for(uint32_t i = 0; i < imuFramesIdx_.size(); i++)
        {
            Eigen::Matrix4d const tformIMU = model.pncData_.oMf[imuFramesIdx_[i]].toHomogeneousMatrix();
            Eigen::Matrix3d const rotIMU = tformIMU.topLeftCorner<3,3>();
            quaternion_t const quatIMU(rotIMU);
            pinocchio::Motion motionIMU = pinocchio::getFrameVelocity(model.pncModel_, 
                                                                      model.pncData_, 
                                                                      imuFramesIdx_[i]);
            Eigen::Vector3d omegaIMU = motionIMU.angular();
            IMUs(0,i) = quatIMU.w();
            IMUs(1,i) = quatIMU.x();
            IMUs(2,i) = quatIMU.y();
            IMUs(3,i) = quatIMU.z();
            IMUs.block<3,1>(4,i) = omegaIMU;
        }

        // Compute command and internal dynamics
        controller_->compute_efforts(engine, t, q, v, u);

        // Compute dynamics
        ddqFull = pinocchio::aba(model.pncModel_, model.pncData_, q, v, u, fext);

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
 
    vectorN_t const & Engine::contactDynamics(int32_t const & frameId) const
    {
        /* /!\ Note that the contact dynamics depends only on kinematics data. /!\ */

        contactOptions_t const * const contactOptions_ = &mdlOptions_->contacts;
        
        Eigen::Matrix4d tformFrame = model.pncData_.oMf[frameId].toHomogeneousMatrix();
        Eigen::Vector3d posFrame = tformFrame.topRightCorner<3,1>();

        vectorN_t fextLocal = vectorN_t::Zero(6);

        if(posFrame(2) < 0.0)
        {
            // Get various transformations
            Eigen::Matrix4d const tformFrame2Jt = model.pncModel_.frames[frameId].placement.toHomogeneousMatrix();
            Eigen::Vector3d fextInWorld(0.0,0.0,0.0);
            Eigen::Vector3d const posFrameJoint = tformFrame2Jt.topRightCorner<3,1>();
            pinocchio::Motion const motionFrame = pinocchio::getFrameVelocity(model.pncModel_, 
                                                                              model.pncData_,
                                                                              frameId);
            Eigen::Vector3d const vFrameInWorld = tformFrame.topLeftCorner<3,3>()*motionFrame.linear();

            // Compute normal force
            float64_t damping = 0;
            if(vFrameInWorld(2)<0)
            {
                damping = -contactOptions_->damping * vFrameInWorld(2);
            }
            fextInWorld(2) = -contactOptions_->stiffness * posFrame(2) + damping;

            // std::cout << contactOptions_->damping << std::endl;
            // std::cout << contactOptions_->stiffness << std::endl;

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
}