#include <iostream>

#include "exo_simu/core/Utilities.h"
#include "exo_simu/core/Model.h"
#include "exo_simu/core/AbstractController.h"


namespace exo_simu
{
    AbstractController::AbstractController(void) :
    ctrlOptions_(nullptr),
    model_(nullptr),
    isInitialized_(false),
    isTelemetryConfigured_(false),
    ctrlOptionsHolder_(),
    telemetrySender_(),
    registeredInfo_()
    {
        AbstractController::setOptions(getDefaultOptions()); // Clarify that the base implementation is called
    }

    AbstractController::~AbstractController(void)
    {
        // Empty.
    }


    result_t AbstractController::initialize(Model const & model)
    {
        result_t returnCode = result_t::SUCCESS;

        if (!model.getIsInitialized())
        {
            std::cout << "Error - AbstractController::initialize - The model is not initialized." << std::endl;
            returnCode = result_t::ERROR_INIT_FAILED;
        }

        if (returnCode == result_t::SUCCESS)
        {
            model_ = &model;

            try
            {
                // isInitialized_ must be true to execute the 'computeCommand' and 'internalDynamics' methods
                isInitialized_ = true;
                float64_t t = 0;
                vectorN_t q = vectorN_t::Zero(model_->nq());
                vectorN_t v = vectorN_t::Zero(model_->nv());
                vectorN_t uCommand = vectorN_t::Zero(model_->getJointsVelocityIdx().size());
                vectorN_t uInternal = vectorN_t::Zero(model_->nv());
                returnCode = computeCommand(t, q, v, uCommand);
                if (returnCode == result_t::SUCCESS)
                {
                    if(uCommand.size() != (int32_t) model_->getJointsVelocityIdx().size())
                    {
                        std::cout << "Error - AbstractController::initialize - 'computeCommand' returns command with wrong size." << std::endl;
                        returnCode = result_t::ERROR_BAD_INPUT;
                    }
                }
                internalDynamics(t, q, v, uInternal); // It cannot fail at this point
                if (returnCode == result_t::SUCCESS)
                {
                    if(uInternal.size() != model_->nv())
                    {
                        std::cout << "Error - AbstractController::initialize - 'internalDynamics' returns command with wrong size." << std::endl;
                        returnCode = result_t::ERROR_BAD_INPUT;
                    }
                }
            }
            catch (std::exception& e)
            {
                std::cout << "Error - AbstractController::initialize - Something is wrong, probably because of 'commandFct'." << std::endl;
                returnCode = result_t::ERROR_GENERIC;
            }
            isInitialized_ = false;
        }

        if (returnCode == result_t::SUCCESS)
        {
            isInitialized_ = true;
        }

        return returnCode;
    }

    void AbstractController::reset(bool const & resetTelemetry)
    {
        // Reset the telemetry state
        if (resetTelemetry)
        {
            registeredInfo_.clear();
            isTelemetryConfigured_ = false;
        }
    }

    result_t AbstractController::configureTelemetry(std::shared_ptr<TelemetryData> const & telemetryData)
    {
        result_t returnCode = result_t::SUCCESS;

        if (!getIsInitialized())
        {
            std::cout << "Error - AbstractController::configureTelemetry - The controller is not initialized." << std::endl;
            returnCode = result_t::ERROR_INIT_FAILED;
        }

        if (returnCode == result_t::SUCCESS)
        {
            if (telemetryData)
            {
                if (ctrlOptions_->telemetryEnable)
                {
                    telemetrySender_.configureObject(telemetryData, CONTROLLER_OBJECT_NAME);
                    for (std::pair<std::string, float64_t const *> const & registeredVariable : registeredInfo_)
                    {
                        (void) telemetrySender_.registerNewEntry<float64_t>(registeredVariable.first, *registeredVariable.second);
                    }
                    isTelemetryConfigured_ = true;
                }
            }
            else
            {
                std::cout << "Error - AbstractController::configureTelemetry - Telemetry not initialized. Impossible to log controller data." << std::endl;
                returnCode = result_t::ERROR_INIT_FAILED;
            }
        }

        return returnCode;
    }

    result_t AbstractController::registerNewVectorEntry(std::vector<std::string> const & fieldNames,
                                                        Eigen::Ref<vectorN_t>    const & values)
    {
        // Delayed variable registration (Taken into account by 'configureTelemetry')

        result_t returnCode = result_t::SUCCESS;

        if (getIsTelemetryConfigured())
        {
            std::cout << "Error - AbstractController::registerNewVectorEntry - Telemetry already initialized. Impossible to register new variables." << std::endl;
            returnCode = result_t::ERROR_INIT_FAILED;
        }

        if (returnCode == result_t::SUCCESS)
        {
            std::vector<std::string>::const_iterator fieldIt = fieldNames.begin();
            for (uint32_t i=0; fieldIt != fieldNames.end(); ++fieldIt, ++i)
            {
                registeredInfo_.emplace_back(*fieldIt, values.data() + i);
            }
        }

        return returnCode;
    }

    result_t AbstractController::registerNewEntry(std::string const & fieldName,
                                                  float64_t   const & value)
    {
        // Delayed variable registration (Taken into account by 'configureTelemetry')

        result_t returnCode = result_t::SUCCESS;

        if (getIsTelemetryConfigured())
        {
            std::cout << "Error - AbstractController::registerNewEntry - Telemetry already initialized. Impossible to register new variables." << std::endl;
            returnCode = result_t::ERROR_INIT_FAILED;
        }

        if (returnCode == result_t::SUCCESS)
        {
            registeredInfo_.emplace_back(fieldName, &value);
        }

        return returnCode;
    }

    void AbstractController::updateTelemetry(void)
    {
        if (getIsTelemetryConfigured())
        {
            for (std::pair<std::string, float64_t const *> const & registeredVariable : registeredInfo_)
            {
                telemetrySender_.updateValue(registeredVariable.first, *registeredVariable.second);
            }
        }
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