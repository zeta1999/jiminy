#include <iostream>

#include "exo_simu/core/AbstractSensor.h"
#include "exo_simu/core/Model.h"
#include "exo_simu/core/Logger.h"

namespace exo_simu
{
    Logger::Logger(void) :
    isInitialized_(false),
    data_(),
    headers_(),
    headersSizes_(),
    sensorsPtr_(nullptr)
    {
        // Empty.
    }

    Logger::~Logger(void)
    {
        // Empty.
    }

    result_t Logger::initialize(Model const & model)
    {
        result_t returnCode = result_t::SUCCESS;

        sensorsMap_t const * sensors = model.getSensorsPtr();

        for (sensorsMap_t::value_type const & kv : *sensors)
        {
            if (returnCode == result_t::SUCCESS)
            {
                if (!kv.second->getIsInitialized())
                {
                    std::cout << "Error - Logger::init - At least one sensor is not initialized." << std::endl;
                    returnCode = result_t::ERROR_INIT_FAILED;
                }
            }
        }

        if (returnCode == result_t::SUCCESS)
        {
            data_.str(std::string());
            headers_.str(std::string());
            headersSizes_.resize(0);
            std::vector<std::string> headersBuffer;
            for(sensorsMap_t::value_type const & kv : *sensors)
            {
                std::vector<std::string> const & sensorHeaders = kv.second->getHeaders();
                headersSizes_.push_back(sensorHeaders.size());
                headersBuffer.insert(headersBuffer.end(), sensorHeaders.begin(), sensorHeaders.end());
            }
            fetchStringVector(headersBuffer, headers_);

            sensorsPtr_ = std::shared_ptr<sensorsMap_t const>(sensors, [](sensorsMap_t const *){}); // Must specify the null deleter

            isInitialized_ = true;
        }

        return returnCode;
    }

    std::string Logger::get(bool const & isHeaderEnable, 
                            bool const & isDataEnable) const
    {
        std::stringstream log;
        if (isHeaderEnable)
        {
            log << headers_.rdbuf() << std::endl;
        }
        if (isDataEnable)
        {
            log << data_.rdbuf();
        }
        return log.str();
    }

    result_t Logger::fetch(void)
    {
        result_t returnCode = result_t::SUCCESS;

        if (!isInitialized_)
        {
            std::cout << "Error - Logger::fetch - Logger not initialized. Impossible to fetch sensors data." << std::endl;
            returnCode = result_t::ERROR_INIT_FAILED;
        }

        std::vector<std::string> dataBuffer;
        if (returnCode == result_t::SUCCESS)
        {
            std::vector<uint32_t>::iterator dataSizeIt = headersSizes_.begin();
            for (sensorsMap_t::value_type const & kv : *sensorsPtr_)
            {
                if (kv.second->sensorOptions_->isLoggingEnable)
                {
                    std::vector<std::string> const & sensorDataString = kv.second->getDataStrings();
                    if (sensorDataString.size() != *dataSizeIt)
                    {
                        std::cout << "Error - Logger::fetch - The sensor data size is different from the header size. Impossible to fetch sensors data." << std::endl;
                        return result_t::ERROR_GENERIC;
                    }
                    dataBuffer.insert(dataBuffer.end(), sensorDataString.begin(), sensorDataString.end());
                    dataSizeIt++;
                }
            }
        }
        if (returnCode == result_t::SUCCESS)
        {
            fetchStringVector(dataBuffer, data_);
        }

        return returnCode;
    }

    void Logger::clear(void)
    {
        data_.str(std::string());
    }

    bool Logger::getIsInitialized(void) const
    {
        return isInitialized_;
    }

    void Logger::fetchStringVector(std::vector<std::string> const & stringsVector, 
                                   std::stringstream              & stream)
    {
        for (std::string const & string : stringsVector)
        {
            if (&string != &stringsVector[0]) {
                stream << logger::SEPARATOR;
            }
            stream << string;
        }
    }
}