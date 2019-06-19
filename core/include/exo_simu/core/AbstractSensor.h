#ifndef SIMU_ABSTRACT_SENSOR_H
#define SIMU_ABSTRACT_SENSOR_H

#include "exo_simu/core/Types.h"


namespace exo_simu
{
    /* T data_ attribute is not defined to avoid having to define another level of 
       abstraction template<typename T> class AbstractSensorTpl: public AbstractSensor */

    class Engine;

    class AbstractSensor
    {
        friend class Engine;

    public:
        virtual configHolder_t getDefaultOptions(void)
        {
            configHolder_t config;
            config["isLoggingEnable"] = true;

            return config;
        };

        struct abstractSensorOptions_t
        {
            bool const isLoggingEnable;

            abstractSensorOptions_t(configHolder_t const & options) :
            isLoggingEnable(boost::get<bool>(options.at("isLoggingEnable")))
            {
                // Empty.
            }
        };

    public:
        AbstractSensor(std::string              const & name, 
                       std::vector<std::string> const & headersSuffix);
        virtual ~AbstractSensor(void);
        virtual AbstractSensor* clone(void) = 0;

        configHolder_t getOptions(void) const;
        void setOptions(configHolder_t const & sensorOptions);
        bool getIsInitialized(void) const;
        std::string getName(void) const;
        std::vector<std::string> getHeaders(void) const;

        virtual std::vector<std::string> const & getDataStrings(void) = 0;

    protected:
        // It assumes that the engine internal state is consistent with other input arguments
        virtual result_t set(Engine    const & engine,
                             float64_t const & t,
                             vectorN_t const & q,
                             vectorN_t const & v,
                             vectorN_t const & a,
                             vectorN_t const & u) = 0;

    public:
        std::shared_ptr<abstractSensorOptions_t const> sensorOptions_;

    private:
        std::string name_;
        std::vector<std::string> headersSuffix_;

    protected:
        bool isInitialized_;
        configHolder_t sensorOptionsHolder_;
    };
}

#endif //end of SIMU_ABSTRACT_SENSOR_H