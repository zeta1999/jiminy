
#include <fstream>
#include <exception>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"

#include "exo_simu/core/TelemetryData.h"
#include "exo_simu/core/Model.h"


namespace exo_simu
{
    Model::Model(void) :
    pncModel_(),
    pncData_(pncModel_),
    mdlOptions_(nullptr),
    contactForces_(),
    isInitialized_(false),
    isTelemetryConfigured_(false),
    urdfPath_(),
    mdlOptionsHolder_(),
    telemetryData_(nullptr),
    sensorsGroupHolder_(),
    contactFramesNames_(),
    jointsNames_(),
    contactFramesIdx_(),
    jointsPositionIdx_(),
    jointsVelocityIdx_(),
    sensorsDataHolder_(),
    nq_(0),
    nv_(0),
    nx_(0)
    {
        setOptions(getDefaultOptions());
    }

    Model::~Model(void)
    {
        // Empty.
    }

    result_t Model::initialize(std::string              const & urdfPath,
                               std::vector<std::string> const & contactFramesNames,
                               std::vector<std::string> const & jointsNames,
                               bool const & hasFreeflyer)
    {
        result_t returnCode = result_t::SUCCESS;

        // Initialize the URDF model
        removeSensors();
        contactFramesNames_ = contactFramesNames;
        jointsNames_ = jointsNames;
        returnCode = setUrdfPath(urdfPath, hasFreeflyer);

        // Update the bounds if necessary
        if (returnCode == result_t::SUCCESS)
        {
            nq_ = pncModel_.nq;
            nv_ = pncModel_.nv;
            nx_ = nq_ + nv_;
            isInitialized_ = true;
            returnCode = setOptions(mdlOptionsHolder_);
        }

        // Extract some joint and frame indices
        if (returnCode == result_t::SUCCESS)
        {
            returnCode = getFramesIdx(contactFramesNames_, contactFramesIdx_);
            contactForces_ = pinocchio::container::aligned_vector<pinocchio::Force>(contactFramesNames_.size(),
                                                                                    pinocchio::Force::Zero());
        }
        if (returnCode == result_t::SUCCESS)
        {
            returnCode = getJointsIdx(jointsNames_, jointsPositionIdx_, jointsVelocityIdx_);
        }

        return returnCode;
    }

    result_t Model::configureTelemetry(std::shared_ptr<TelemetryData> const & telemetryData)
    {
        telemetryData_ = std::shared_ptr<TelemetryData>(telemetryData);

        return result_t::SUCCESS;
    }

    result_t Model::removeSensor(std::string const & name)
    {
        // Can be used to remove either a sensor or a sensor type

        result_t returnCode = result_t::SUCCESS;

        sensorsGroupHolder_t::iterator sensorGroupIt = sensorsGroupHolder_.find(name);
        if (sensorGroupIt != sensorsGroupHolder_.end())
        {
            sensorsGroupHolder_.erase(sensorGroupIt);
            sensorsDataHolder_.erase(name);
        }
        else
        {
            bool isSensorDeleted = false;
            for (sensorsGroupHolder_t::value_type & sensorGroup : sensorsGroupHolder_)
            {
                sensorsHolder_t::iterator sensorIt = sensorGroup.second.find(name);
                if (sensorIt != sensorGroup.second.end())
                {
                    // Remove the sensor from its group
                    sensorGroup.second.erase(sensorIt);

                    // Remove the sensor group if there is no more sensors left.
                    if (sensorGroup.second.empty())
                    {
                        sensorsGroupHolder_.erase(sensorGroup.first);
                        sensorsDataHolder_.erase(sensorGroup.first);
                    }

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
        sensorsGroupHolder_.clear();
        sensorsDataHolder_.clear();
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
                if((int32_t) jointsPositionIdx_.size() != boundsMin.size() || (uint32_t) jointsPositionIdx_.size() != boundsMax.size())
                {
                    std::cout << "Error - Model::setOptions - Wrong vector size for boundsMin or boundsMax." << std::endl;
                    returnCode = result_t::ERROR_BAD_INPUT;
                }
            }
        }

        mdlOptions_ = std::make_unique<modelOptions_t const>(mdlOptionsHolder_);

        return returnCode;
    }

    bool Model::getIsInitialized(void) const
    {
        return isInitialized_;
    }

    bool Model::getIsTelemetryConfigured(void) const
    {
        return isTelemetryConfigured_;
    }

    std::string Model::getUrdfPath(void) const
    {
        return urdfPath_;
    }

    result_t Model::setUrdfPath(std::string const & urdfPath, bool const & hasFreeflyer)
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
                if (hasFreeflyer)
                {
                    pinocchio::urdf::buildModel(urdfPath, pinocchio::JointModelFreeFlyer(), modelEmpty);
                }
                else
                {
                    pinocchio::urdf::buildModel(urdfPath, modelEmpty);
                }
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

    matrixN_t const & Model::getSensorsData(std::string const & sensorType) const
    {
        return sensorsGroupHolder_.at(sensorType).begin()->second->getAll();
    }

    matrixN_t::ConstRowXpr Model::getSensorData(std::string const & sensorType,
                                                std::string const & sensorName) const
    {
        return sensorsGroupHolder_.at(sensorType).at(sensorName)->get();
    }

    void Model::setSensorsData(float64_t const & t,
                               vectorN_t const & q,
                               vectorN_t const & v,
                               vectorN_t const & a,
                               vectorN_t const & u)
    {
        for (sensorsGroupHolder_t::value_type const & sensorGroup : sensorsGroupHolder_)
        {
            if (!sensorGroup.second.empty())
            {
                sensorGroup.second.begin()->second->setAll(t, q, v, a, u); // Access static member of the sensor Group through the first instance
            }
        }
    }

    void Model::updateSensorsTelemetry(void)
    {
        for (sensorsGroupHolder_t::value_type const & sensorGroup : sensorsGroupHolder_)
        {
            if (!sensorGroup.second.empty())
            {
                sensorGroup.second.begin()->second->updateTelemetryAll(); // Access static member of the sensor Group through the first instance
            }
        }
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
                                int32_t           & jointPositionIdx,
                                int32_t           & jointVelocityIdx) const
    {
        // It only return the index of the first element if the joint has multiple degrees of freedom

        /* Obtained using the cumulative sum of number of variables for each joint previous
            to the desired one in the kinematic chain but skipping the first joint, which
            is always the "universe" and is irrelevant (enforced by pinocchio itself). */

        result_t returnCode = result_t::SUCCESS;

        if (!pncModel_.existJointName(jointName))
        {
            std::cout << "Error - ExoModel::getFrameIdx - Frame '" << jointName << "' not found in urdf." << std::endl;
            returnCode = result_t::ERROR_BAD_INPUT;
        }

        if (returnCode == result_t::SUCCESS)
        {
            int32_t jointModelIdx = pncModel_.getJointId(jointName);
            jointPositionIdx = 0;
            jointVelocityIdx = 0;
            for (auto jointIt = pncModel_.joints.begin() + 1; jointIt != pncModel_.joints.begin() + jointModelIdx; jointIt++)
            {
                jointPositionIdx += jointIt->nq();
                jointVelocityIdx += jointIt->nv();
            }
        }

        return returnCode;
    }

    result_t Model::getJointsIdx(std::vector<std::string> const & jointsNames,
                                 std::vector<int32_t>           & jointsPositionIdx,
                                 std::vector<int32_t>           & jointsVelocityIdx) const
    {
        result_t returnCode = result_t::SUCCESS;

        jointsPositionIdx.resize(0);
        jointsVelocityIdx.resize(0);
        for (std::string const & name : jointsNames)
        {
            if (returnCode == result_t::SUCCESS)
            {
                int32_t positionIdx;
                int32_t velocityIdx;
                returnCode = getJointIdx(name, positionIdx, velocityIdx);
                jointsPositionIdx.push_back(positionIdx);
                jointsVelocityIdx.push_back(velocityIdx);
            }
        }

        return returnCode;
    }

    uint32_t Model::nq(void) const
    {
        return nq_;
    }

    uint32_t Model::nv(void) const
    {
        return nv_;
    }

    uint32_t Model::nx(void) const
    {
        return nx_;
    }
}
