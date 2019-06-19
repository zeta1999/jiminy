#ifndef SIMU_MODEL_H
#define SIMU_MODEL_H

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/frames.hpp"

#include "exo_simu/core/Types.h"


namespace exo_simu
{
    class AbstractSensor;

    class Model
    {
    protected:
        virtual configHolder_t getDefaultJointOptions()
        {
            configHolder_t config;
            config["boundsFromUrdf"] = true; // Must be true since boundsMin and boundsMax are undefined
            config["boundsMin"] = vectorN_t();
            config["boundsMax"] = vectorN_t();

            return config;
        };

        struct jointOptions_t
        {
            bool      const boundsFromUrdf;
            vectorN_t const boundsMin;
            vectorN_t const boundsMax;

            jointOptions_t(configHolder_t const & options):
            boundsFromUrdf(boost::get<bool>(options.at("boundsFromUrdf"))),
            boundsMin(boost::get<vectorN_t>(options.at("boundsMin"))),
            boundsMax(boost::get<vectorN_t>(options.at("boundsMax")))
            {
                // Empty.
            }
        };

    public:
        virtual configHolder_t getDefaultOptions()
        {
            configHolder_t config;
            config["joints"] = getDefaultJointOptions();

            return config;
        };

        struct modelOptions_t
        {
            jointOptions_t const joints;

            modelOptions_t(configHolder_t const & options):
            joints(boost::get<configHolder_t>(options.at("joints")))
            {
                // Empty.
            }
        };

    public:
        Model(void);
        virtual ~Model(void);

        result_t initialize(std::string          const & urdfPath, 
                            std::vector<int32_t> const & contactFramesIdx);

        result_t addSensor(AbstractSensor * sensor);
        result_t removeSensor(std::string const & sensorName);
        void removeSensors(void);

        configHolder_t getOptions(void) const;
        result_t setOptions(configHolder_t const & mdlOptions);
        bool getIsInitialized(void) const;
        std::string getUrdfPath(void) const;
        sensorsMap_t const * getSensorsPtr(void) const;
        std::vector<int32_t> const & getContactFramesIdx(void) const;

    protected:
        result_t setUrdfPath(std::string const & urdfPath);
        result_t getFrameIdx(std::string const & frameName, 
                             int32_t           & frameIdx) const;
        result_t getFramesIdx(std::vector<std::string> const & framesNames, 
                              std::vector<int32_t>           & framesIdx) const;
        result_t getJointIdx(std::string const & jointName, 
                             int32_t           & jointIdx) const;
        result_t getJointsIdx(std::vector<std::string> const & jointsNames, 
                              std::vector<int32_t>           & jointsIdx) const;
        
    public:
        pinocchio::Model pncModel_;
        pinocchio::Data pncData_;
        std::shared_ptr<modelOptions_t const> mdlOptions_;

    protected:
        bool isInitialized_;
        std::string urdfPath_;
        configHolder_t mdlOptionsHolder_;
        sensorsMap_t sensors_;

        std::vector<int32_t> contactFramesIdx_;
    };
}

#endif //end of SIMU_MODEL_H