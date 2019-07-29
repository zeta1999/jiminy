#ifndef WDC_EXO_MODEL_H
#define WDC_EXO_MODEL_H

#include <cmath>

#include "jiminy/core/Model.h"


namespace jiminy
{
namespace wdc
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

            exoJointOptions_t(configHolder_t const & options) :
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
            // Just proxies to gather all the telemetry parameters at the same place

            configHolder_t config;
            config["enableForceSensors"] = true;
            config["enableImuSensors"] = true;
            config["enableEncoderSensors"] = true;

            return config;
        };

        struct exoTelemetryOptions_t
        {
            bool const enableForceSensors;
            bool const enableImuSensors;
            bool const enableEncoderSensors;

            exoTelemetryOptions_t(configHolder_t const & options) :
            enableForceSensors(boost::get<bool>(options.at("enableForceSensors"))),
            enableImuSensors(boost::get<bool>(options.at("enableImuSensors"))),
            enableEncoderSensors(boost::get<bool>(options.at("enableEncoderSensors")))
            {
                // Empty.
            }
        };

        virtual configHolder_t getDefaultOptions() override
        {
            configHolder_t config;
            config["dynamics"] = getDefaultDynamicsOptions();
            config["joints"] = getDefaultJointOptions();
            config["telemetry"] = getDefaultTelemetryOptions();

            return config;
        };

        struct exoModelOptions_t : public modelOptions_t
        {
            exoJointOptions_t     const joints;     // Shadowing on purpose
            exoTelemetryOptions_t const telemetry;

            exoModelOptions_t(configHolder_t const & options) :
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

        result_t initialize(std::string              const & urdfPath,
                            std::vector<std::string> const & contactFramesNames,
                            std::vector<std::string> const & motorsNames,
                            bool                     const & hasFreeflyer) = delete;
        result_t initialize(std::string const & urdfPath);
        virtual void reset(void) override;

        result_t setOptions(configHolder_t const & mdlOptions);
        std::vector<int32_t> const & getToesVelocityIdx(void) const;

    protected:
        result_t loadUrdfModel(std::string const & urdfPath) = delete;

    public:
        std::unique_ptr<exoModelOptions_t const> exoMdlOptions_;

    private:
        std::vector<std::string> toesNames_;
        std::vector<int32_t> toesVelocityIdx_; // Indices of the toes in the velocity vector representation
    };
}
}

#endif //end of WDC_EXO_MODEL_H