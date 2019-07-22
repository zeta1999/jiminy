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
        ImuSensor(Model                               const & model,
                  std::shared_ptr<SensorDataHolder_t> const & dataHolder,
                  std::string                         const & name);
        ~ImuSensor(void);

        void initialize(int32_t const & frameIdx);

        int32_t getFrameIdx(void) const;

    protected:
        result_t set(float64_t const & t,
                     vectorN_t const & q,
                     vectorN_t const & v,
                     vectorN_t const & a,
                     vectorN_t const & u) override;

    private:
        int32_t frameIdx_;
    };

    class ForceSensor : public AbstractSensorTpl<ForceSensor>
    {
    public:
        ForceSensor(Model                               const & model,
                    std::shared_ptr<SensorDataHolder_t> const & dataHolder,
                    std::string                         const & name);
        ~ForceSensor(void);

        void initialize(int32_t const & frameIdx);

        int32_t getFrameIdx(void) const;

    protected:
        result_t set(float64_t const & t,
                     vectorN_t const & q,
                     vectorN_t const & v,
                     vectorN_t const & a,
                     vectorN_t const & u);

    private:
        int32_t frameIdx_;
    };

    class EncoderSensor : public AbstractSensorTpl<EncoderSensor>
    {
    public:
        EncoderSensor(Model                               const & model,
                      std::shared_ptr<SensorDataHolder_t> const & dataHolder,
                      std::string                         const & name);
        ~EncoderSensor(void);

        void initialize(int32_t const & motorPositionIdx,
                        int32_t const & motorVelocityIdx);

        int32_t getMotorPositionIdx(void) const;
        int32_t getMotorVelocityIdx(void) const;

    protected:
        result_t set(float64_t const & t,
                     vectorN_t const & q,
                     vectorN_t const & v,
                     vectorN_t const & a,
                     vectorN_t const & u);

    private:
        int32_t motorPositionIdx_;
        int32_t motorVelocityIdx_;
    };
}

#endif //end of SIMU_STANDARD_SENSORS_H