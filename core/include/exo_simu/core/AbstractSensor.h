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
        static configHolder_t getDefaultOptions(void)
        {
            configHolder_t config;
            config["enablePostProccess"] = true;
            config["noiseStd"] = 0.0;
            config["biasMean"] = 0.0;
            config["biasStd"] = 0.0;

            return config;
        };

        virtual configHolder_t getDefaultInstanceOptions(void)
        {
            return getDefaultOptions();
        };

        struct abstractSensorOptions_t
        {
            bool const enablePostProccess;
            float64_t const noiseStd;
            float64_t const biasMean;
            float64_t const biasStd;

            abstractSensorOptions_t(configHolder_t const & options) :
            enablePostProccess(boost::get<bool>(options.at("enablePostProccess"))),
            noiseStd(boost::get<float64_t>(options.at("noiseStd"))),
            biasMean(boost::get<float64_t>(options.at("biasMean"))),
            biasStd(boost::get<float64_t>(options.at("biasStd")))
            {
                // Empty.
            }
        };

    public:
        // Disable the copy of the class
        AbstractSensorBase(AbstractSensorBase const & abstractSensor) = delete;
        AbstractSensorBase & operator = (AbstractSensorBase const & other) = delete;

    public:
        AbstractSensorBase(Model       const & model,
                           std::string const & name);
        virtual ~AbstractSensorBase(void);

        virtual void reset(void);
        virtual result_t configureTelemetry(std::shared_ptr<TelemetryData> const & telemetryData);

        bool getIsInitialized(void) const;
        bool getIsTelemetryConfigured(void) const;
        virtual std::string getName(void) const;
        virtual std::vector<std::string> const & getFieldNames(void) const = 0;
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
        virtual matrixN_t::RowXpr data(void) = 0;

        virtual result_t set(float64_t const & t,
                             vectorN_t const & q,
                             vectorN_t const & v,
                             vectorN_t const & a,
                             vectorN_t const & u) = 0;

    private:
        std::string name_;

    protected:
        TelemetrySender telemetrySender_;
        bool isInitialized_;
        bool isTelemetryConfigured_;
        Model const * model_; // Must be a pointer to avoid managing its deletion
    };

    template<class T>
    class AbstractSensorTpl : public AbstractSensorBase
    {
    public:
        using AbstractSensorBase::getDefaultOptions;

    public:
        // Disable the copy of the class
        AbstractSensorTpl(AbstractSensorTpl const & abstractSensor) = delete;
        AbstractSensorTpl & operator = (AbstractSensorTpl const & other)  = delete;

    public:
        AbstractSensorTpl(Model                               const & model,
                          std::shared_ptr<SensorDataHolder_t> const & dataHolder,
                          std::string                         const & name);
        virtual ~AbstractSensorTpl(void);

        virtual void reset(void) override;

        static configHolder_t getOptions(void);
        static void setOptions(configHolder_t const & sensorOptions);
        std::vector<std::string> const & getFieldNames(void) const;
        vectorN_t const & getBias(void) const;

        matrixN_t::ConstRowXpr get(void) const override;
        matrixN_t const & getAll(void) const override;
        result_t setAll(float64_t const & t,
                        vectorN_t const & q,
                        vectorN_t const & v,
                        vectorN_t const & a,
                        vectorN_t const & u) override;
        void updateTelemetryAll(void) override;

    protected:
        virtual matrixN_t::RowXpr data(void) override;

    public:
        static std::string const type_;
        static uint32_t const sizeOf_;
        static std::vector<std::string> const fieldNamesPostProcess_;
        static std::vector<std::string> const fieldNamesPreProcess_;
        static std::unique_ptr<abstractSensorOptions_t const> sensorOptions_;

    protected:
        static configHolder_t sensorOptionsHolder_;

    private:
        std::shared_ptr<SensorDataHolder_t> dataHolder_;
        uint32_t sensorId_;
        vectorN_t bias;
    };
}

#include "exo_simu/core/AbstractSensor.tcc"

#endif //end of SIMU_ABSTRACT_SENSOR_H