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
            config["boundsFromUrdf"] = true;
            config["boundsMin"] = (vectorN_t) (-M_PI * vectorN_t::Ones(14));
            config["boundsMax"] = (vectorN_t) ( M_PI * vectorN_t::Ones(14));
            config["frictionViscous"] = (vectorN_t) (0*(vectorN_t(14) << 100.0,100.0,100.0,100.0,20.0,20.0,0.0,
                                                                         100.0,100.0,100.0,100.0,20.0,20.0,0.0).finished());
            config["frictionDry"] = (vectorN_t) (0*(vectorN_t(14) << 10.0,10.0,10.0,10.0,2.0,2.0,0.0,
                                                                     10.0,10.0,10.0,10.0,2.0,2.0,0.0).finished());
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
        Model* clone(void);

        result_t initialize(std::string const & urdfPath);

        result_t setOptions(configHolder_t const & mdlOptions);

    protected:
        result_t setUrdfPath(std::string const & urdfPath);
        result_t getFrameIdx(std::string const & frameName, 
                             int32_t           & frameIdx) const;
        result_t getFramesIdx(std::vector<std::string> const & framesNames, 
                              std::vector<int32_t>           & framesIdx) const;
        result_t getJointIdx(std::string const & jointName, 
                             int32_t           & jointPositionIdx, 
                             int32_t           & jointVelocityIdx) const;
        result_t getJointsIdx(std::vector<std::string> const & jointsNames, 
                              std::vector<int32_t>           & jointsPositionIdx, 
                              std::vector<int32_t>           & jointsVelocityIdx) const;
        
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
    };
}

#endif //end of WDC_EXO_MODEL_H