#include "exo_simu/core/Model.h"
#include "exo_simu/core/Utilities.h"
#include "exo_simu/wdc/ExoModel.h"
#include "exo_simu/wdc/Controller.h"

namespace exo_simu
{
    Controller::Controller(void) :
    AbstractController()
    {
        // Empty.
    }

    Controller::~Controller(void)
    {
        // Empty.
    }

    AbstractController* Controller::clone(void)
    {
        return new Controller(*this);
    }

    result_t Controller::initialize(void)
    {
        result_t returnCode = result_t::SUCCESS;

        // TODO
        isInitialized_ = true;

        return returnCode;
    }

    void Controller::compute_command(float64_t const & t,
                                     vectorN_t const & q,
                                     vectorN_t const & v,
                                     vectorN_t       & u)
    {
        u = -(Kp.array() * q.segment(7,12).array() + Kd.array() * v.segment(6,12).array());
    }

    void Controller::internalDynamics(Engine    const & engine,
                                      float64_t const & t,
                                      vectorN_t const & q,
                                      vectorN_t const & v,
                                      vectorN_t       & u)
    {
        ExoModel const & exoModel = static_cast<ExoModel const &>(engine.model);
        ExoModel::exoJointOptions_t const * jointOptions_ = &(exoModel.exoMdlOptions_->joints); // Pointer for consistency

        // Joint friction
        for (uint32_t i = 0; i < exoModel.jointsIdx_.size(); i++)
        {
            u(i+6) = -jointOptions_->frictionViscous(i)*dq(i+6) - jointOptions_->frictionDry(i) * \
                     saturateSoft(dq(i+6) / jointOptions_->dryFrictionVelEps,-1.0,1.0,0.7);
        }

        // Joint bounds
        for (uint32_t i = 0; i<exoModel.nu_; i++)
        {
            float64_t const qJt = q(i+7);
            float64_t const dqJt = dq(i+6);
            float64_t const qJtMin = jointOptions_->boundsMin(i);
            float64_t const qJtMax = jointOptions_->boundsMax(i);

            float64_t fJt;
            if (qJt > qJtMax)
            {
                float64_t const qErr = qJt - qJtMax;
                float64_t damping = 0;
                if (dqJt > 0)
                {
                    damping = -jointOptions_->boundDamping*dqJt;
                }

                fJt = -jointOptions_->boundStiffness * qErr + damping;

                float64_t blendingFactor = qErr / jointOptions_->boundTransitionEps;
                if (blendingFactor>1.0)
                {
                    blendingFactor = 1.0;
                }

                fJt *= blendingFactor;
                u(i + 6) += fJt;
            }

            if (qJt < qJtMin)
            {
                float64_t const qErr = qJtMin - qJt;
                float64_t damping = 0;
                if (dqJt < 0)
                {
                    damping = -jointOptions_->boundDamping * dqJt;
                }

                fJt = jointOptions_->boundStiffness * qErr + damping;

                float64_t blendingFactor = qErr / jointOptions_->boundTransitionEps;
                if (blendingFactor>1.0)
                {
                    blendingFactor = 1.0;
                }

                fJt *= blendingFactor;
                u(i + 6) += fJt;
            }
        }
    }
}