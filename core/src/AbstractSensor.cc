#include "exo_simu/core/Model.h"
#include "exo_simu/core/AbstractSensor.h"


namespace exo_simu
{
    AbstractSensorBase::AbstractSensorBase(Model       const & model,
                                           std::string const & name) :
    sensorOptions_(nullptr),
    name_(name),
    fieldNames_(),
    telemetrySender_(),
    isInitialized_(false),
    isTelemetryConfigured_(false),
    model_(&model),                 
    sensorOptionsHolder_()
    {
        AbstractSensorBase::setOptions(getDefaultOptions()); // Clarify that the base implementation is called
    }

    AbstractSensorBase::AbstractSensorBase(AbstractSensorBase const & abstractSensor) :
    sensorOptions_(nullptr),
    name_(abstractSensor.name_),
    fieldNames_(abstractSensor.fieldNames_),
    telemetrySender_(abstractSensor.telemetrySender_),
    isInitialized_(abstractSensor.isInitialized_),
    isTelemetryConfigured_(abstractSensor.isTelemetryConfigured_),
    model_(abstractSensor.model_),                 
    sensorOptionsHolder_(abstractSensor.sensorOptionsHolder_)
    {
        AbstractSensorBase::setOptions(abstractSensor.sensorOptionsHolder_);
    }

    AbstractSensorBase & AbstractSensorBase::operator = (AbstractSensorBase const & other) 
    {
        if (this != &other)
        {
            AbstractSensorBase * temp(other.clone());
            std::swap(sensorOptions_, temp->sensorOptions_);
            std::swap(name_, temp->name_);
            std::swap(fieldNames_, temp->fieldNames_);
            std::swap(telemetrySender_, temp->telemetrySender_);
            std::swap(isInitialized_, temp->isInitialized_),
            std::swap(isTelemetryConfigured_, temp->isTelemetryConfigured_);
            std::swap(model_, temp->model_);  
            std::swap(sensorOptionsHolder_, temp->sensorOptionsHolder_);
            delete temp;
        }
        return *this;
    }

    AbstractSensorBase::~AbstractSensorBase(void)
    {
        // Empty.
    }

    result_t AbstractSensorBase::configureTelemetry(std::vector<std::string>       const & fieldNames,
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

    configHolder_t AbstractSensorBase::getOptions(void) const
    {
        return sensorOptionsHolder_;
    }

    void AbstractSensorBase::setOptions(configHolder_t const & sensorOptions)
    {
        sensorOptionsHolder_ = sensorOptions;
        sensorOptions_ = std::make_shared<abstractSensorOptions_t const>(sensorOptionsHolder_);
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
            updateVectorValue(telemetrySender_, fieldNames_, get());
        }
    }
}