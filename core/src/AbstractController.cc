#include <iostream>

#include <boost/algorithm/clamp.hpp>

#include "exo_simu/core/Model.h"
#include "exo_simu/core/AbstractController.h"

namespace exo_simu
{
    AbstractController::AbstractController(void) :
    ctrlOptions_(nullptr),
    isInitialized_(false),
    ctrlOptionsHolder_()
    {
        AbstractController::setOptions(getDefaultOptions()); // Clarify that the base implementation is called
    }

    AbstractController::~AbstractController(void)
    {
        // Empty.
    }
                                        
    result_t AbstractController::compute_efforts(Model     const & model,
                                                 float64_t const & t,
                                                 vectorN_t const & q,
                                                 vectorN_t const & v,
                                                 vectorN_t       & u)
    {
        result_t returnCode = result_t::SUCCESS;
        
        if (!isInitialized_)
        {
            std::cout << "Error - AbstractController::compute_efforts - Controller not initialized. Impossible to compute efforts." << std::endl;
            returnCode = result_t::ERROR_INIT_FAILED;
        }

        if (returnCode == result_t::SUCCESS)
        {
            std::vector<int32_t> jointsVelocityIdx = model.getJointsVelocityIdx();
            vectorN_t uCommand = vectorN_t::Zero(jointsVelocityIdx.size());
            compute_command(model, t, q, v, uCommand); //TODO: Send the values at the previous iteration
            vectorN_t uInternal = vectorN_t::Zero(model.nv());
            internalDynamics(model, t, q, v, uInternal);
            u = uInternal;
            for (uint32_t i=0; i < jointsVelocityIdx.size(); i++)
            {
                uint32_t jointId = jointsVelocityIdx[i];
                float64_t torque_max = model.pncModel_.effortLimit(jointId); // effortLimit is given in the velocity vector space
                u[jointId] += boost::algorithm::clamp(uCommand[i], -torque_max, torque_max);
            }
        }
        
        return returnCode;
    }

    configHolder_t AbstractController::getOptions(void) const
    {
        return ctrlOptionsHolder_;
    }

    void AbstractController::setOptions(configHolder_t const & ctrlOptions)
    {
        ctrlOptionsHolder_ = ctrlOptions;
        ctrlOptions_ = std::make_shared<controllerOptions_t const>(ctrlOptionsHolder_);
    }

    bool AbstractController::getIsInitialized(void) const
    {
        return isInitialized_;
    }
}