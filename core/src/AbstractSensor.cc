#include "exo_simu/core/Model.h"
#include "exo_simu/core/AbstractSensor.h"


namespace exo_simu
{
    AbstractSensor::AbstractSensor(Model       const & model,
                                   std::string const & name) :
    sensorOptions_(nullptr),
    name_(name),
    isInitialized_(false),
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

    std::string AbstractSensor::getName(void) const
    {
        return name_;
    }
}