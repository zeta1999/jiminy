#ifndef WDC_EXO_MODEL_H
#define WDC_EXO_MODEL_H

#include <cmath>

#include "exo_simu/core/Model.h"


namespace exo_simu
{
    class ExoModel : public Model
    {
        friend class Controller;

    public:
        virtual configHolder_t getDefaultJointOptions() override
        {
            // Add extra options or update default values
            configHolder_t config = Model::getDefaultJointOptions();
            config["boundsFromUrdf"] = true;
            config["boundsMin"] = (vectorN_t) (-M_PI * vectorN_t::Ones(12));
            config["boundsMax"] = (vectorN_t) ( M_PI * vectorN_t::Ones(12));
            config["frictionViscous"] = (vectorN_t) (0*(vectorN_t(12) << 100.0,100.0,100.0,100.0,20.0,20.0,
                                                                         100.0,100.0,100.0,100.0,20.0,20.0).finished());
            config["frictionDry"] = (vectorN_t) (0*(vectorN_t(12) << 10.0,10.0,10.0,10.0,2.0,2.0,
                                                                     10.0,10.0,10.0,10.0,2.0,2.0).finished());
            config["dryFrictionVelEps"] = 1.0e-2;
            return config;
        };

        struct exoJointOptions_t : public jointOptions_t
        {
            vectorN_t const frictionViscous;
            vectorN_t const frictionDry;
            float64_t const dryFrictionVelEps;

            exoJointOptions_t(configHolder_t const & options):
            jointOptions_t(options),
            frictionViscous(boost::get<vectorN_t>(options.at("frictionViscous"))),
            frictionDry(boost::get<vectorN_t>(options.at("frictionDry"))),
            dryFrictionVelEps(boost::get<float64_t>(options.at("dryFrictionVelEps")))
            {
                // Empty.
            }
        };

        virtual configHolder_t getDefaultTelemetryOptions()
        {
            configHolder_t config;
            config["logForceSensors"] = true;
            config["logImuSensors"] = true;
            config["logEncoderSensors"] = true;
            return config;
        };

        struct exoTelemetryOptions_t
        {
            bool const logForceSensors;
            bool const logImuSensors;
            bool const logEncoderSensors;

            exoTelemetryOptions_t(configHolder_t const & options):
            logForceSensors(boost::get<bool>(options.at("logForceSensors"))),
            logImuSensors(boost::get<bool>(options.at("logImuSensors"))),
            logEncoderSensors(boost::get<bool>(options.at("logEncoderSensors")))
            {
                // Empty.
            }
        };

        virtual configHolder_t getDefaultOptions() override
        {
            configHolder_t config;
            config["joints"] = getDefaultJointOptions();
            config["telemetry"] = getDefaultTelemetryOptions();

            return config;
        };

        struct exoModelOptions_t : public modelOptions_t
        {
            exoJointOptions_t const joints; // Hide the original property of modelOptions_t
            exoTelemetryOptions_t const telemetry;

            exoModelOptions_t(configHolder_t const & options):
            modelOptions_t(options),
            joints(boost::get<configHolder_t>(options.at("joints"))),
            telemetry(boost::get<configHolder_t>(options.at("telemetry")))
            {
                // Empty.
            }
        };

    public:
        ExoModel(void);
        ~ExoModel(void);

        result_t initialize(std::string          const & urdfPath, 
                            std::vector<int32_t> const & contactFramesIdx) = delete;
        result_t initialize(std::string const & urdfPath);

        virtual configHolder_t getSensorsOptions(void) const override;
        virtual void setSensorsOptions(configHolder_t & sensorOptions) override;
        result_t setOptions(configHolder_t const & mdlOptions);

    protected:
        virtual result_t configureTelemetry(std::shared_ptr<TelemetryData> const & telemetryData) override;

        result_t setUrdfPath(std::string const & urdfPath);

    public:
        std::unique_ptr<exoModelOptions_t const> exoMdlOptions_;
    };
}

#endif //end of WDC_EXO_MODEL_H