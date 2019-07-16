#include <iostream>

#include "exo_simu/core/Model.h"
#include "exo_simu/core/AbstractController.h"


namespace exo_simu
{
    AbstractController::AbstractController(void) :
    ctrlOptions_(nullptr),
    isInitialized_(false),
    isTelemetryConfigured_(false),
    ctrlOptionsHolder_(),
    telemetrySender_()
    {
        AbstractController::setOptions(getDefaultOptions()); // Clarify that the base implementation is called
    }

    AbstractController::~AbstractController(void)
    {
        // Empty.
    }

    void AbstractController::reset(void)
    {
        // Empty.
    }

    result_t AbstractController::configureTelemetry(std::shared_ptr<TelemetryData> const & telemetryData)
    {
        result_t returnCode = result_t::SUCCESS; 

        if (telemetryData)
        {
            if (ctrlOptions_->telemetryEnable)
            {
                telemetrySender_.configureObject(telemetryData, CONTROLLER_OBJECT_NAME);
                isTelemetryConfigured_ = true;
            }
        }
        else
        {
            std::cout << "Error - AbstractController::configureTelemetry - Telemetry not initialized. Impossible to log controller data." << std::endl;
            returnCode = result_t::ERROR_INIT_FAILED;
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
        ctrlOptions_ = std::make_unique<controllerOptions_t const>(ctrlOptionsHolder_);
    }

    bool AbstractController::getIsInitialized(void) const
    {
        return isInitialized_;
    }

    bool AbstractController::getIsTelemetryConfigured(void) const
    {
        return isTelemetryConfigured_;
    }
}