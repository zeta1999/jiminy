#ifndef SIMU_ABSTRACT_SENSOR_H
#define SIMU_ABSTRACT_SENSOR_H

#include "exo_simu/core/Types.h"


namespace exo_simu
{
    class Engine;

    class AbstractSensor
    {
        friend class Engine;

    public:
        virtual configHolder_t getDefaultOptions(void)
        {
            configHolder_t config;
            // No default configuration parameter

            return config;
        };

        struct abstractSensorOptions_t
        {
            abstractSensorOptions_t(configHolder_t const & options)
            {
                // Empty.
            }
        };

    public:
        AbstractSensor(std::string const & name);
        virtual ~AbstractSensor(void);
        virtual AbstractSensor* clone(void) = 0;

        configHolder_t getOptions(void) const;
        void setOptions(configHolder_t const & sensorOptions);
        bool getIsInitialized(void) const;
        std::string getName(void) const;
        vectorN_t get(void) const;

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

    protected:
        bool isInitialized_;
        /* vectorN_t data_ attribute only to avoid having to define another level of 
        abstraction template<typename T> class AbstractSensorTpl: public AbstractSensor */
        vectorN_t data_;
        configHolder_t sensorOptionsHolder_;
    };
}

#endif //end of SIMU_ABSTRACT_SENSOR_H