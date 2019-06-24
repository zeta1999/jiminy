#ifndef SIMU_STANDARD_SENSORS_H
#define SIMU_STANDARD_SENSORS_H

#include <iostream>

#include "exo_simu/core/AbstractSensor.h"


namespace exo_simu
{
    class Engine;

    class ImuSensor : public AbstractSensor
    {
    public:
        configHolder_t getDefaultOptions(void)
        {
            configHolder_t config = AbstractSensor::getDefaultOptions();
            // No extra configuration parameter

            return config;
        };
        
        struct imuSensorOptions_t : public abstractSensorOptions_t
        {
            imuSensorOptions_t(configHolder_t const & options):
            abstractSensorOptions_t(options)
            {
                // Empty.
            }
        };

    public:
        ImuSensor(std::string const & name);
        ~ImuSensor(void);
        AbstractSensor* clone(void);

        void initialize(int32_t const & framesIdx);

        void setOptions(configHolder_t const & sensorOptions);
        int32_t getFrameIdx(void) const;

    protected:
        result_t set(Engine    const & engine,
                     float64_t const & t,
                     vectorN_t const & q,
                     vectorN_t const & v,
                     vectorN_t const & a,
                     vectorN_t const & u);

    public:
        std::shared_ptr<imuSensorOptions_t const> imuSensorOptions_;

    private:
        int32_t framesIdx_;
    };

    class ForceSensor : public AbstractSensor
    {
    public:
        configHolder_t getDefaultOptions(void)
        {
            configHolder_t config = AbstractSensor::getDefaultOptions();
            // No extra configuration parameter

            return config;
        };
        
        struct forceSensorOptions_t : public abstractSensorOptions_t
        {
            forceSensorOptions_t(configHolder_t const & options):
            abstractSensorOptions_t(options)
            {
                // Empty.
            }
        };

    public:
        ForceSensor(std::string const & name);
        ~ForceSensor(void);
        AbstractSensor* clone(void);

        void initialize(int32_t const & framesIdx);

        void setOptions(configHolder_t const & sensorOptions);
        int32_t getFrameIdx(void) const;
        
    protected:
        result_t set(Engine    const & engine,
                     float64_t const & t,
                     vectorN_t const & q,
                     vectorN_t const & v,
                     vectorN_t const & a,
                     vectorN_t const & u);

    public:
        std::shared_ptr<forceSensorOptions_t const> forceSensorOptions_;

    private:
        int32_t framesIdx_;
    };
}

#endif //end of SIMU_STANDARD_SENSORS_H