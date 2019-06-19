
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
    contactFramesIdx_()
    {
        setOptions(getDefaultOptions());
    }

    Model::~Model(void)
    {
        // Empty.
    }

    result_t Model::initialize(std::string          const & urdfPath, 
                               std::vector<int32_t> const & contactFramesIdx)
    {
        result_t returnCode = result_t::SUCCESS;

        removeSensors();
        contactFramesIdx_ = contactFramesIdx;
        returnCode = setUrdfPath(urdfPath);

        if (returnCode == result_t::SUCCESS)
        {
            isInitialized_ = true;
            returnCode = setOptions(mdlOptionsHolder_); // Update the bounds if necessary
        }

        return returnCode;
    }

    result_t Model::addSensor(AbstractSensor * sensor)
    {
        result_t returnCode = result_t::SUCCESS;

        std::string sensorName = sensor->getName();

        sensorsMap_t::iterator it = sensors_.find(sensorName);
        if (it != sensors_.end())
        {
            std::cout << "Error - Model::addSensor - Sensor with the same name already exists." << std::endl;
            returnCode = result_t::ERROR_BAD_INPUT;
        }

        if (returnCode == result_t::SUCCESS)
        {
            sensors_[sensorName] = std::shared_ptr<AbstractSensor>(sensor->clone());
        }

        return returnCode;
    }

    result_t Model::removeSensor(std::string const & sensorName)
    {
        result_t returnCode = result_t::SUCCESS;

        sensorsMap_t::iterator it = sensors_.find(sensorName);
        if (it == sensors_.end())
        {
            std::cout << "Error - Model::removeSensor - Sensor with this name does not exist." << std::endl;
            returnCode = result_t::ERROR_BAD_INPUT;
        }

        if (returnCode == result_t::SUCCESS)
        {
            sensors_.erase(it);
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
        pncModel_.gravity = boost::get<vectorN_t>(mdlOptions.at("gravity"));

        configHolder_t& jointOptionsHolder_ = boost::get<configHolder_t>(mdlOptionsHolder_.at("joints"));
        bool boundsFromUrdf = boost::get<bool>(jointOptionsHolder_.at("boundsFromUrdf"));
        vectorN_t & boundsMin = boost::get<vectorN_t>(jointOptionsHolder_.at("boundsMin"));
        vectorN_t & boundsMax = boost::get<vectorN_t>(jointOptionsHolder_.at("boundsMax"));
        if (isInitialized_)
        {
            if (boundsFromUrdf)
            {
                boundsMin = pncModel_.lowerPositionLimit;
                boundsMax = pncModel_.upperPositionLimit;
            }
            else
            {
                if(pncModel_.nq != boundsMin.size() || pncModel_.nq != boundsMax.size())
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
 
    sensorsMap_t const * Model::getSensorsPtr(void) const
    {
        return &sensors_;
    }

    std::vector<int32_t> const & Model::getContactFramesIdx(void) const
    {
        return contactFramesIdx_;
    }

    result_t Model::getFrameIdx(std::string const & frameName, 
                                int32_t           & frameIdx) const
    {
        result_t returnCode = result_t::SUCCESS;

        if (!pncModel_.existFrame(frameName))
        {
            std::cout << "Error - ExoModel::getFrameIdx - Frame '" << frameName << "' not found in urdf." << std::endl;
            returnCode = result_t::ERROR_BAD_INPUT;
        }

        if (returnCode == result_t::SUCCESS)
        {
            frameIdx = pncModel_.getFrameId(frameName);
        }

        return returnCode;
    }

    result_t Model::getFramesIdx(std::vector<std::string> const & framesNames, 
                                 std::vector<int32_t>           & framesIdx) const
    {
        result_t returnCode = result_t::SUCCESS;

        framesIdx.resize(0);
        for (std::string const & name : framesNames)
        {
            if (returnCode == result_t::SUCCESS)
            {
                int32_t idx;
                returnCode = getFrameIdx(name, idx);
                framesIdx.push_back(idx);
            }
        }

        return returnCode;
    }

    result_t Model::getJointIdx(std::string const & jointName, 
                                int32_t           & jointIdx) const
    {
        result_t returnCode = result_t::SUCCESS;

        if (!pncModel_.existJointName(jointName))
        {
            std::cout << "Error - ExoModel::getFrameIdx - Frame '" << jointName << "' not found in urdf." << std::endl;
            returnCode = result_t::ERROR_BAD_INPUT;
        }

        if (returnCode == result_t::SUCCESS)
        {
            jointIdx = pncModel_.getFrameId(jointName);
        }

        return returnCode;
    }

    result_t Model::getJointsIdx(std::vector<std::string> const & jointsNames, 
                                 std::vector<int32_t>           & jointsIdx) const
    {
        result_t returnCode = result_t::SUCCESS;

        jointsIdx.resize(0);
        for (std::string const & name : jointsNames)
        {
            if (returnCode == result_t::SUCCESS)
            {
                int32_t idx;
                returnCode = getJointIdx(name, idx);
                jointsIdx.push_back(idx);
            }
        }

        return returnCode;
    }
}