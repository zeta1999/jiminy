#include "exo_simu/core/AbstractSensor.h"

namespace exo_simu
{
    AbstractSensor::AbstractSensor(std::string              const & name, 
                                   std::vector<std::string> const & headersSuffix) :
    sensorOptions_(nullptr),
    name_(name),
    headersSuffix_(headersSuffix),
    isInitialized_(false),
    sensorOptionsHolder_()
    {
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
        sensorOptions_ = std::shared_ptr<abstractSensorOptions_t const>(new abstractSensorOptions_t(sensorOptionsHolder_));
    }

    bool AbstractSensor::getIsInitialized(void) const
    {
        return isInitialized_;
    }

    std::string AbstractSensor::getName(void) const
    {
        return name_;
    }

    std::vector<std::string> AbstractSensor::getHeaders(void) const
    {
        std::vector<std::string> headers;
        for (std::string const & suffix : headersSuffix_) 
        {
            headers.push_back(name_ + "_" + suffix);
        }

        return headers;
    }
}