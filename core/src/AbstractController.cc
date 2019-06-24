#include <iostream>
#include "exo_simu/core/Engine.h"
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
                                        
    result_t AbstractController::compute_efforts(Engine    const & engine,
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
            std::vector<int32_t> jointsVelocityIdx = engine.getModel().getJointsVelocityIdx();
            vectorN_t uCommand = vectorN_t::Zero(jointsVelocityIdx.size());
            compute_command(engine, t, q, v, uCommand);
            vectorN_t uInternal = vectorN_t::Zero(engine.nv());
            internalDynamics(engine, t, q, v, uInternal);
            u = uInternal;
            for (uint32_t i=0; i < jointsVelocityIdx.size(); i++)
            {
                u[jointsVelocityIdx[i]] += uCommand[i];
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
        ctrlOptions_ = std::shared_ptr<controllerOptions_t const>(new controllerOptions_t(ctrlOptionsHolder_));
    }

    bool AbstractController::getIsInitialized(void) const
    {
        return isInitialized_;
    }
}