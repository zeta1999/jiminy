#include "exo_simu/core/Model.h"
#include "exo_simu/core/AbstractSensor.h"


namespace exo_simu
{
    AbstractSensorBase::AbstractSensorBase(Model       const & model,
                                           std::string const & name) :
    name_(name),
    telemetrySender_(),
    isInitialized_(false),
    isTelemetryConfigured_(false),
    model_(&model)
    {
        // Empty.
    }

    AbstractSensorBase::~AbstractSensorBase(void)
    {
        // Empty.
    }

    result_t AbstractSensorBase::configureTelemetry(std::shared_ptr<TelemetryData> const & telemetryData)
    {
        result_t returnCode = result_t::SUCCESS; 

        if (telemetryData)
        {
            telemetrySender_.configureObject(telemetryData, name_);
            (void) registerNewVectorEntry(telemetrySender_, getFieldNames(), get());
            isTelemetryConfigured_ = true;
        }
        else
        {
            std::cout << "Error - AbstractSensorTpl::configureTelemetry - Telemetry not initialized. Impossible to log sensor data." << std::endl;
            returnCode = result_t::ERROR_INIT_FAILED;
        }

        return returnCode;
    }

    bool AbstractSensorBase::getIsInitialized(void) const
    {
        return isInitialized_;
    }

    bool AbstractSensorBase::getIsTelemetryConfigured(void) const
    {
        return isTelemetryConfigured_;
    }

    std::string AbstractSensorBase::getName(void) const
    {
        return name_;
    }

    void AbstractSensorBase::updateTelemetry(void)
    {
        if(getIsTelemetryConfigured())
        {
            updateVectorValue(telemetrySender_, getFieldNames(), get());
        }
    }
}