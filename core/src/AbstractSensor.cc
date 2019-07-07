#include "exo_simu/core/Model.h"
#include "exo_simu/core/AbstractSensor.h"


namespace exo_simu
{
    AbstractSensor::AbstractSensor(Model       const & model,
                                   std::string const & name) :
    sensorOptions_(nullptr),
    name_(name),
    fieldNames_(),
    telemetrySender_(),
    isInitialized_(false),
    isTelemetryConfigured_(false),
    model_(),                 
    sensorOptionsHolder_()
    {
        model_ = std::make_shared<Model>(model);
        AbstractSensor::setOptions(getDefaultOptions()); // Clarify that the base implementation is called
    }

    AbstractSensor::~AbstractSensor(void)
    {
        // Empty.
    }

    result_t AbstractSensor::configureTelemetry(std::vector<std::string>       const & fieldNames,
                                                std::shared_ptr<TelemetryData> const & telemetryData)
    {
        result_t returnCode = result_t::SUCCESS; 

        if (telemetryData)
        {
            fieldNames_ = fieldNames;
            telemetrySender_.configureObject(telemetryData, name_);
            (void) registerNewVectorEntry(telemetrySender_, fieldNames_, get());
            isTelemetryConfigured_ = true;
        }
        else
        {
            std::cout << "Error - AbstractSensorTpl::configureTelemetry - Telemetry not initialized. Impossible to log sensor data." << std::endl;
            returnCode = result_t::ERROR_INIT_FAILED;
        }

        return returnCode;
    }

    configHolder_t AbstractSensor::getOptions(void) const
    {
        return sensorOptionsHolder_;
    }

    void AbstractSensor::setOptions(configHolder_t const & sensorOptions)
    {
        sensorOptionsHolder_ = sensorOptions;
        sensorOptions_ = std::make_shared<abstractSensorOptions_t const>(sensorOptionsHolder_);
    }

    bool AbstractSensor::getIsInitialized(void) const
    {
        return isInitialized_;
    }

    bool AbstractSensor::getIsTelemetryConfigured(void) const
    {
        return isTelemetryConfigured_;
    }

    std::string AbstractSensor::getName(void) const
    {
        return name_;
    }

    void AbstractSensor::updateTelemetry(void)
    {
        if(getIsTelemetryConfigured())
        {
            updateVectorValue(telemetrySender_, fieldNames_, get());
        }
    }
}