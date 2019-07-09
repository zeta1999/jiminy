#ifndef SIMU_STANDARD_SENSORS_H
#define SIMU_STANDARD_SENSORS_H

#include <iostream>

#include "exo_simu/core/AbstractSensor.h"


namespace exo_simu
{
    class Model;

    class ImuSensor : public AbstractSensorTpl<ImuSensor>
    {
    public:
        configHolder_t getDefaultOptions(void) override
        {
            configHolder_t config = AbstractSensorTpl<ImuSensor>::getDefaultOptions();
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
        ImuSensor(Model                               const & model,
                  std::shared_ptr<SensorDataHolder_t> const & dataHolder,
                  std::string                         const & name);
        ~ImuSensor(void);
        AbstractSensorBase* clone(void) const override;

        void initialize(int32_t const & frameIdx);

        void setOptions(configHolder_t const & sensorOptions);
        int32_t getFrameIdx(void) const;

    protected:
        result_t set(float64_t const & t,
                     vectorN_t const & q,
                     vectorN_t const & v,
                     vectorN_t const & a,
                     vectorN_t const & u) override;

    public:
        std::shared_ptr<imuSensorOptions_t const> imuSensorOptions_;

    private:
        int32_t frameIdx_;
    };

    class ForceSensor : public AbstractSensorTpl<ForceSensor>
    {
    public:
        configHolder_t getDefaultOptions(void) override
        {
            configHolder_t config = AbstractSensorTpl<ForceSensor>::getDefaultOptions();
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
        ForceSensor(Model                               const & model,
                    std::shared_ptr<SensorDataHolder_t> const & dataHolder,
                    std::string                         const & name);
        ~ForceSensor(void);
        AbstractSensorBase* clone(void) const override;

        void initialize(int32_t const & frameIdx);

        void setOptions(configHolder_t const & sensorOptions);
        int32_t getFrameIdx(void) const;
        
    protected:
        result_t set(float64_t const & t,
                     vectorN_t const & q,
                     vectorN_t const & v,
                     vectorN_t const & a,
                     vectorN_t const & u);

    public:
        std::shared_ptr<forceSensorOptions_t const> forceSensorOptions_;

    private:
        int32_t frameIdx_;
    };

    class EncoderSensor : public AbstractSensorTpl<EncoderSensor>
    {
    public:
        configHolder_t getDefaultOptions(void) override
        {
            configHolder_t config = AbstractSensorTpl<EncoderSensor>::getDefaultOptions();
            // No extra configuration parameter

            return config;
        };
        
        struct encoderSensorOptions_t : public abstractSensorOptions_t
        {
            encoderSensorOptions_t(configHolder_t const & options):
            abstractSensorOptions_t(options)
            {
                // Empty.
            }
        };

    public:
        EncoderSensor(Model                               const & model,
                      std::shared_ptr<SensorDataHolder_t> const & dataHolder,
                      std::string                         const & name);
        ~EncoderSensor(void);
        AbstractSensorBase* clone(void) const override;

        void initialize(int32_t const & jointPositionIdx,
                        int32_t const & jointVelocityIdx);

        void setOptions(configHolder_t const & sensorOptions);
        int32_t getJointPositionIdx(void) const;
        int32_t getJointVelocityIdx(void) const;
        
    protected:
        result_t set(float64_t const & t,
                     vectorN_t const & q,
                     vectorN_t const & v,
                     vectorN_t const & a,
                     vectorN_t const & u);

    public:
        std::shared_ptr<encoderSensorOptions_t const> encoderSensorOptions_;

    private:
        int32_t jointPositionIdx_;
        int32_t jointVelocityIdx_;
    };
}

#endif //end of SIMU_STANDARD_SENSORS_H