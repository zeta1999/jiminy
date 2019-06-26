#ifndef SIMU_ABSTRACT_SENSOR_H
#define SIMU_ABSTRACT_SENSOR_H

#include "exo_simu/core/Types.h"

namespace exo_simu
{
    class Model;

    class AbstractSensor
    {
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
        virtual matrixN_t::ConstRowXpr const get(void) const = 0;
        virtual matrixN_t const & getAll(void) = 0;

        // It assumes that the model internal state is consistent with other input arguments
        virtual result_t set(Model     const & model,
                             float64_t const & t,
                             vectorN_t const & q,
                             vectorN_t const & v,
                             vectorN_t const & a,
                             vectorN_t const & u) = 0;
        virtual result_t setAll(Model     const & model,
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
        configHolder_t sensorOptionsHolder_;
    };

    template<class T>
    class SensorDataHolder : public virtual AbstractSensor
    {
    public:
        SensorDataHolder(std::string const & name):
        AbstractSensor(name),
        sensorId_(sensorNb_)
        {
            ++sensorNb_;
            dataHolder_.conservativeResize(sensorNb_, sizeOf_);
            data() = vectorN_t::Zero(sizeOf_);
            sensorsHolder_.push_back(this);
            sensorsCopyCounter_.push_back(1);
        }

        SensorDataHolder(SensorDataHolder const & sensorDataHolder):
        AbstractSensor(sensorDataHolder),
        sensorId_(sensorDataHolder.sensorId_)
        {
            /* Do NOT create a new sensor and making a copy
               BUT update the pointer in the global container
               in case the original object would be destroyed. */

            sensorsHolder_[sensorId_] = this;
            ++sensorsCopyCounter_[sensorId_];
        }

        ~SensorDataHolder(void)
        {
            --sensorsCopyCounter_[sensorId_];
            if (!sensorsCopyCounter_[sensorId_])
            {
                // Remove associated row in the global data buffer
                if(sensorId_ < sensorNb_ - 1)
                {
                    dataHolder_.block(sensorId_, 0, sensorNb_ - sensorId_ - 1, sizeOf_) = dataHolder_.bottomRows(sensorNb_ - sensorId_ - 1).eval(); // eval to avoid aliasing
                }
                dataHolder_.conservativeResize(sensorNb_ - 1, sizeOf_);

                // Shift the sensor ids
                for (uint32_t i=sensorId_ + 1; i < sensorNb_; i++)
                {
                    --sensorsHolder_[i]->sensorId_;
                }

                // Remove the deprecated elements of the global containers
                sensorsHolder_.erase(sensorsHolder_.begin() + sensorId_);
                sensorsCopyCounter_.erase(sensorsCopyCounter_.begin() + sensorId_);

                // Update the total number of sensors left
                --sensorNb_;
            }
        };

        SensorDataHolder & operator = (SensorDataHolder const & other) 
        {
            if (this != &other) {
                /* Make the overriden sensor floating by deleting the
                   data in the global containers. Note that it does
                   NOT actually destroy the object. */
                ~SensorDataHolder();

                /* Assign the same sensor id to the floating sensor
                   than the other one, so that they share the same
                   data in the global container. */
                AbstractSensor::operator=(other);
                sensorId_ = other.sensorId;
                sensorsHolder_[sensorId_] = this;
                ++sensorsCopyCounter_[sensorId_];
            }
            
            return *this;
        } 

        matrixN_t::ConstRowXpr const get(void) const override
        {
            return Eigen::Block<matrixN_t const, 1, Eigen::Dynamic>(dataHolder_.derived(), sensorId_, 0, 1, sizeOf_); // Const version of matrixN_t::row method
        };

        matrixN_t const & getAll(void)
        {
            return dataHolder_;
        };

        result_t setAll(Model     const & model,
                        float64_t const & t,
                        vectorN_t const & q,
                        vectorN_t const & v,
                        vectorN_t const & a,
                        vectorN_t const & u) override
        {
            result_t returnCode = result_t::SUCCESS; 

            for (SensorDataHolder<T> * sensor : sensorsHolder_)
            {
                if (returnCode == result_t::SUCCESS)
                {
                    returnCode = sensor->set(model, t, q, v, a, u);
                }
            }

            return returnCode;
        };

    protected:
        matrixN_t::RowXpr data(void)
        {
            return dataHolder_.row(sensorId_);
        };

    private:
        static std::vector<uint32_t> sensorsCopyCounter_;
        static std::vector<SensorDataHolder<T> *> sensorsHolder_;
        static matrixN_t dataHolder_;
        static uint32_t const sizeOf_;
        static uint32_t sensorNb_;
        uint32_t sensorId_;
    };
}

#endif //end of SIMU_ABSTRACT_SENSOR_H