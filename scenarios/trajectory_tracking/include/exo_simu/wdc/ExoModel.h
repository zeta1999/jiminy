#ifndef WDC_EXO_MODEL_H
#define WDC_EXO_MODEL_H

#include <cmath>

#include "exo_simu/core/Types.h"
#include "exo_simu/core/Model.h"


namespace exo_simu
{
    class ExoModel : public Model
    {
        friend class Controller;

    public:
        configHolder_t getDefaultJointOptions()
        {
            // Add extra options or update default values
            configHolder_t config = Model::getDefaultJointOptions();
            config["boundsFromUrdf"] = false; 
            config["boundsMin"] = (vectorN_t(21) << -std::numeric_limits<float64_t>::infinity() * vectorN_t::Ones(7), 
                                                    -M_PI * vectorN_t::Ones(14)).finished();
            config["boundsMax"] = (vectorN_t(21) <<  std::numeric_limits<float64_t>::infinity() * vectorN_t::Ones(7),  
                                                     M_PI * vectorN_t::Ones(14)).finished();
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

        struct exoModelOptions_t : public modelOptions_t
        {
            exoJointOptions_t const joints; // Hide the property of modelOptions_t

            exoModelOptions_t(configHolder_t const & options):
            modelOptions_t(options),
            joints(boost::get<configHolder_t>(options.at("joints")))
            {
                // Empty.
            }
        };

        ExoModel(void);
        ~ExoModel(void);

        result_t initialize(std::string const & urdfPath);

        result_t setOptions(configHolder_t const & mdlOptions);

    private:
        // Discourage usage of the base initializer
        result_t initialize(std::string          const & urdfPath, 
                            std::vector<int32_t> const & contactFramesIdx)
        {
            std::cout << "Error - ExoModel::initialize - Base method not available from derived class." << std::endl;
            return result_t::ERROR_GENERIC;
        }

    public:
        std::shared_ptr<exoModelOptions_t const> exoMdlOptions_;
        
    private:
        std::vector<std::string> contactFramesNames_;
        std::vector<std::string> imuFramesNames_;
        std::vector<std::string> jointsNames_;

        std::vector<int32_t> imuFramesIdx_;
        std::vector<int32_t> jointsIdx_;
    };
}

#endif //end of WDC_EXO_MODEL_H