#ifndef SIMU_ABSTRACT_SENSOR_H
#define SIMU_ABSTRACT_SENSOR_H

#include "exo_simu/core/Types.h"
#include "exo_simu/core/Utilities.h"
#include "exo_simu/core/TelemetrySender.h"
#include "exo_simu/core/Model.h"


namespace exo_simu
{    
    class TelemetryData;
    template<typename> class AbstractSensorTpl;

    class AbstractSensorBase
    {
        template<typename> friend class AbstractSensorTpl;

    public:
        // Disable the copy of the class
        AbstractSensorBase(AbstractSensorBase const & abstractSensor) = delete;
        AbstractSensorBase & operator = (AbstractSensorBase const & other) = delete;

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
        AbstractSensorBase(Model       const & model,
                           std::string const & name);
        virtual ~AbstractSensorBase(void);
        friend void swap(AbstractSensorBase & first, 
                         AbstractSensorBase & second);

        virtual result_t configureTelemetry(std::vector<std::string>       const & fieldNames,
                                            std::shared_ptr<TelemetryData> const & telemetryData);

        configHolder_t getOptions(void) const;
        virtual void setOptions(configHolder_t const & sensorOptions);
        bool getIsInitialized(void) const;
        bool getIsTelemetryConfigured(void) const;
        std::string getName(void) const;
        virtual matrixN_t::ConstRowXpr get(void) const = 0;
        virtual matrixN_t const & getAll(void) const = 0;

        // It assumes that the model internal state is consistent with other input arguments
        virtual result_t setAll(float64_t const & t,
                                vectorN_t const & q,
                                vectorN_t const & v,
                                vectorN_t const & a,
                                vectorN_t const & u) = 0;
        void updateTelemetry(void);
        virtual void updateTelemetryAll(void) = 0;

    protected:
        virtual result_t set(float64_t const & t,
                             vectorN_t const & q,
                             vectorN_t const & v,
                             vectorN_t const & a,
                             vectorN_t const & u) = 0;

    public:
        std::shared_ptr<abstractSensorOptions_t const> sensorOptions_;

    private:
        std::string name_;

    protected:
        std::vector<std::string> fieldNames_;
        TelemetrySender telemetrySender_;
        bool isInitialized_;
        bool isTelemetryConfigured_;
        Model const * model_; // Must be a pointer to avoid managing its deletion
        configHolder_t sensorOptionsHolder_;
    };

    template<class T>
    class AbstractSensorTpl : public AbstractSensorBase
    {
    public:
        // Disable the copy of the class
        AbstractSensorTpl(AbstractSensorTpl const & abstractSensor) = delete;
        AbstractSensorTpl & operator = (AbstractSensorTpl const & other)  = delete;

    public:
        AbstractSensorTpl(Model                               const & model,
                          std::shared_ptr<SensorDataHolder_t> const & dataHolder,
                          std::string                         const & name):
        AbstractSensorBase(model, name),
        dataHolder_(dataHolder),
        sensorId_(dataHolder_->num_)
        {
            ++dataHolder_->num_;
            dataHolder_->data_.conservativeResize(dataHolder_->num_, sizeOf_);
            data() = vectorN_t::Zero(sizeOf_);
            dataHolder_->sensors_.push_back(this);
            dataHolder_->counters_.push_back(1);
        };

        virtual ~AbstractSensorTpl(void)
        {
            --dataHolder_->counters_[sensorId_];
            if (!dataHolder_->counters_[sensorId_])
            {
                // Remove associated row in the global data buffer
                if(sensorId_ < dataHolder_->num_ - 1)
                {
                    dataHolder_->data_.block(sensorId_, 0, dataHolder_->num_ - sensorId_ - 1, sizeOf_) = 
                        dataHolder_->data_.block(sensorId_ + 1, 0, dataHolder_->num_ - sensorId_ - 1, sizeOf_).eval(); // eval to avoid aliasing
                }
                dataHolder_->data_.conservativeResize(dataHolder_->num_ - 1, sizeOf_);

                // Shift the sensor ids
                for (uint32_t i=sensorId_ + 1; i < dataHolder_->num_; i++)
                {
                    AbstractSensorTpl<T> * sensor = static_cast<AbstractSensorTpl<T> *>(dataHolder_->sensors_[i]);
                    --sensor->sensorId_;
                }

                // Remove the deprecated elements of the global containers
                dataHolder_->sensors_.erase(dataHolder_->sensors_.begin() + sensorId_);
                dataHolder_->counters_.erase(dataHolder_->counters_.begin() + sensorId_);

                // Update the total number of sensors left
                --dataHolder_->num_;
            }
        };

        matrixN_t::ConstRowXpr get(void) const override
        {
            return matrixN_t::ConstRowXpr(dataHolder_->data_.derived(), sensorId_, 0, 1, sizeOf_);
        };

        matrixN_t const & getAll(void) const override
        {
            return dataHolder_->data_;
        };

        result_t setAll(float64_t const & t,
                        vectorN_t const & q,
                        vectorN_t const & v,
                        vectorN_t const & a,
                        vectorN_t const & u) override
        {
            result_t returnCode = result_t::SUCCESS; 

            for (AbstractSensorBase * sensor : dataHolder_->sensors_)
            {
                if (returnCode == result_t::SUCCESS)
                {
                    returnCode = sensor->set(t, q, v, a, u);
                }
            }

            return returnCode;
        };

        void updateTelemetryAll(void) override
        {
            for (AbstractSensorBase * sensor : dataHolder_->sensors_)
            {
                sensor->updateTelemetry();
            }
        };

    protected:
        matrixN_t::RowXpr data(void)
        {
            return matrixN_t::RowXpr(dataHolder_->data_.derived(), sensorId_, 0, 1, sizeOf_);
        };

    public:
        static std::string const type_;
        static uint32_t const sizeOf_;

    private:
        std::shared_ptr<SensorDataHolder_t> dataHolder_;
        uint32_t sensorId_;
    };
}

#endif //end of SIMU_ABSTRACT_SENSOR_H