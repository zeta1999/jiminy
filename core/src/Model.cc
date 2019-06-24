
#include <fstream>
#include <exception>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"

#include "exo_simu/core/AbstractSensor.h"
#include "exo_simu/core/Model.h"

namespace exo_simu
{
    Model::Model(void) :
    pncModel_(),
    pncData_(pncModel_),
    mdlOptions_(nullptr),
    isInitialized_(false),
    urdfPath_(),
    mdlOptionsHolder_(),
    sensors_(),
    contactFramesIdx_(),
    jointsPositionIdx_(),
    jointsVelocityIdx_()
    {
        setOptions(getDefaultOptions());
    }

    Model::~Model(void)
    {
        // Empty.
    }

    Model* Model::clone(void)
    {
        return new Model(*this);
    }

    result_t Model::initialize(std::string          const & urdfPath, 
                               std::vector<int32_t> const & contactFramesIdx, 
                               std::vector<int32_t> const & jointsPositionIdx, 
                               std::vector<int32_t> const & jointsVelocityIdx)
    {
        result_t returnCode = result_t::SUCCESS;

        removeSensors();
        contactFramesIdx_ = contactFramesIdx;
        jointsPositionIdx_ = jointsPositionIdx;
        jointsVelocityIdx_ = jointsVelocityIdx;
        returnCode = setUrdfPath(urdfPath);

        if (returnCode == result_t::SUCCESS)
        {
            isInitialized_ = true;
            returnCode = setOptions(mdlOptionsHolder_); // Update the bounds if necessary
        }

        return returnCode;
    }

    result_t Model::addSensor(std::string    const & sensorType, 
                              AbstractSensor       * sensor)
    {
        // The sensor name must be unique, even if there types are different
        result_t returnCode = result_t::SUCCESS;

        std::string sensorName = sensor->getName();

        for (sensorsGroupMap_t::value_type const & sensorGroup : sensors_)
        {
            sensorsMap_t::const_iterator it = sensorGroup.second.find(sensorName);
            if (it != sensorGroup.second.end())
            {
                std::cout << "Error - Model::addSensor - Sensor with the same name already exists." << std::endl;
                returnCode = result_t::ERROR_BAD_INPUT;
            }
        }

        if (returnCode == result_t::SUCCESS)
        {
            sensors_[sensorType][sensorName] = std::shared_ptr<AbstractSensor>(sensor->clone());
        }

        return returnCode;
    }

    result_t Model::removeSensor(std::string const & sensorName)
    {
        // Can be used to remove either a sensor or a sensor type
        result_t returnCode = result_t::SUCCESS;

        sensorsGroupMap_t::iterator sensorGroupIt = sensors_.find(sensorName);
        if (sensorGroupIt != sensors_.end())
        {
            sensors_.erase(sensorGroupIt);
        }
        else
        {
            bool isSensorDeleted = false;
            for (sensorsGroupMap_t::value_type & sensorGroup : sensors_)
            {
                sensorsMap_t::iterator sensorIt = sensorGroup.second.find(sensorName);
                if (sensorIt != sensorGroup.second.end())
                {
                    sensorGroup.second.erase(sensorIt);
                    isSensorDeleted = true;
                    break;
                }
            }

            if (!isSensorDeleted)
            {
                std::cout << "Error - Model::removeSensor - Sensor with this name does not exist." << std::endl;
                returnCode = result_t::ERROR_BAD_INPUT;
            }
        }

        return returnCode;
    }

    void Model::removeSensors(void)
    {
        sensors_.clear();
    }

    configHolder_t Model::getOptions(void) const
    {
        return mdlOptionsHolder_;
    }

    result_t Model::setOptions(configHolder_t const & mdlOptions)
    {
        result_t returnCode = result_t::SUCCESS;

        mdlOptionsHolder_ = mdlOptions;

        configHolder_t & jointOptionsHolder_ = boost::get<configHolder_t>(mdlOptionsHolder_.at("joints"));
        vectorN_t & boundsMin = boost::get<vectorN_t>(jointOptionsHolder_.at("boundsMin"));
        vectorN_t & boundsMax = boost::get<vectorN_t>(jointOptionsHolder_.at("boundsMax"));
        if (isInitialized_)
        {
            if (boost::get<bool>(jointOptionsHolder_.at("boundsFromUrdf")))
            {
                boundsMin = vectorN_t::Zero(jointsPositionIdx_.size());
                boundsMax = vectorN_t::Zero(jointsPositionIdx_.size());
                for (uint32_t i=0; i < jointsPositionIdx_.size(); i++)
                {
                    boundsMin[i] = pncModel_.lowerPositionLimit[jointsPositionIdx_[i]];
                    boundsMax[i] = pncModel_.upperPositionLimit[jointsPositionIdx_[i]];
                }
            }
            else
            {
                if(jointsPositionIdx_.size() != boundsMin.size() || jointsPositionIdx_.size() != boundsMax.size())
                {
                    std::cout << "Error - Model::setOptions - Wrong vector size for boundsMin or boundsMax." << std::endl;
                    returnCode = result_t::ERROR_BAD_INPUT;
                }
            }
        }

        mdlOptions_ = std::shared_ptr<modelOptions_t const>(new modelOptions_t(mdlOptionsHolder_));

        return returnCode;
    }

    bool Model::getIsInitialized(void) const
    {
        return isInitialized_;
    }

    std::string Model::getUrdfPath(void) const
    {
        return urdfPath_;
    }

    result_t Model::setUrdfPath(std::string const & urdfPath)
    {
        result_t returnCode = result_t::SUCCESS;

        if (!std::ifstream(urdfPath.c_str()).good())
        {
            std::cout << "Error - Model::setUrdfPath - The URDF file does not exist. Impossible to load it." << std::endl;
            returnCode = result_t::ERROR_BAD_INPUT;
        }
        urdfPath_ = urdfPath;

        if (returnCode == result_t::SUCCESS)
        {
            try
            {
                pinocchio::Model modelEmpty;
                pinocchio::urdf::buildModel(urdfPath,pinocchio::JointModelFreeFlyer(),modelEmpty);
                pncModel_ = modelEmpty;
            }
            catch (std::exception& e)
            {
                std::cout << "Error - Model::setUrdfPath - Something is wrong with the URDF. Impossible to build a model from it." << std::endl;
                returnCode = result_t::ERROR_BAD_INPUT;
            }
        }

        if (returnCode == result_t::SUCCESS)
        {
            setOptions(mdlOptionsHolder_); // Make sure bounds are the right ones
        }

        return returnCode;
    }
 
    sensorsGroupMap_t const * Model::getSensors(void) const
    {
        return &sensors_;
    }

    std::vector<int32_t> const & Model::getContactFramesIdx(void) const
    {
        return contactFramesIdx_;
    }

    std::vector<int32_t> const & Model::getJointsPositionIdx(void) const
    {
        return jointsPositionIdx_;
    }

    std::vector<int32_t> const & Model::getJointsVelocityIdx(void) const
    {
        return jointsVelocityIdx_;
    }
}