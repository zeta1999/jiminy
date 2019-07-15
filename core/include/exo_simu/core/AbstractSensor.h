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
        virtual configHolder_t getDefaultOptions(void)
        {
            configHolder_t config;
            config["enablePostProccess"] = true;
            config["noiseStd"] = vectorN_t();
            config["bias"] = vectorN_t();

            return config;
        };

        struct abstractSensorOptions_t
        {
            bool const enablePostProccess;
            vectorN_t const noiseStd;
            rowN_t const bias;
            float64_t const delay;

            abstractSensorOptions_t(configHolder_t const & options) :
            enablePostProccess(boost::get<bool>(options.at("enablePostProccess"))),
            noiseStd(boost::get<vectorN_t>(options.at("noiseStd"))),
            bias(boost::get<vectorN_t>(options.at("bias")))
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

        configHolder_t getOptions(void);
        void setOptions(configHolder_t const & sensorOptions);
        virtual void setOptionsAll(configHolder_t const & sensorOptions) = 0;
        bool getIsInitialized(void) const;
        bool getIsTelemetryConfigured(void) const;
        virtual std::string getName(void) const;
        virtual std::string getType(void) const = 0;
        virtual std::vector<std::string> const & getFieldNames(void) const = 0;

        virtual matrixN_t::ConstRowXpr get(void) const = 0;
        virtual matrixN_t const & getAll(void) const = 0;
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


    public:
        std::unique_ptr<abstractSensorOptions_t const> sensorOptions_;
        
    protected:
        configHolder_t sensorOptionsHolder_;
        TelemetrySender telemetrySender_;
        bool isInitialized_;
        bool isTelemetryConfigured_;
        Model const * model_; // Raw pointer to avoid managing its deletion

    private:
        std::string name_;
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
                          std::string                         const & name);
        virtual ~AbstractSensorTpl(void);

        virtual std::string getType(void) const override;
        std::vector<std::string> const & getFieldNames(void) const;

        virtual void setOptionsAll(configHolder_t const & sensorOptions) override;
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

    private:
        std::shared_ptr<SensorDataHolder_t> dataHolder_;
        uint32_t sensorId_;
    };
}

#include "exo_simu/core/AbstractSensor.tcc"

#endif //end of SIMU_ABSTRACT_SENSOR_H