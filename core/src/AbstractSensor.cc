#include "exo_simu/core/Model.h"
#include "exo_simu/core/AbstractSensor.h"


namespace exo_simu
{
    AbstractSensorBase::AbstractSensorBase(Model       const & model,
                                           std::string const & name) :
    sensorOptions_(),
    sensorOptionsHolder_(),
    telemetrySender_(),
    isInitialized_(false),
    isTelemetryConfigured_(false),
    model_(&model),
    name_(name),
    data_(),
    isDataUpToDate_(false)
    {
        setOptions(getDefaultOptions());
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
            telemetrySender_.configureObject(telemetryData, getTelemetryName());
            (void) registerNewVectorEntry(telemetrySender_, getFieldNames(), data_);
            isTelemetryConfigured_ = true;
        }
        else
        {
            std::cout << "Error - AbstractSensorBase::configureTelemetry - Telemetry not initialized. Impossible to log sensor data." << std::endl;
            returnCode = result_t::ERROR_INIT_FAILED;
        }

        return returnCode;
    }

    configHolder_t AbstractSensorBase::getOptions(void)
    {
        return sensorOptionsHolder_;
    }

    void AbstractSensorBase::setOptions(configHolder_t const & sensorOptions)
    {
        sensorOptionsHolder_ = sensorOptions;
        sensorOptions_ = std::make_unique<abstractSensorOptions_t const>(sensorOptionsHolder_);
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
            get(data_); // Force update the internal buffer data_ if necessary
            updateVectorValue(telemetrySender_, getFieldNames(), data_);
        }
    }
}