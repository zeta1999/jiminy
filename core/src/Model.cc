
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
    hasFreeflyer_(false),
    mdlOptionsHolder_(),
    telemetryData_(nullptr),
    sensorsGroupHolder_(),
    contactFramesNames_(),
    motorsNames_(),
    contactFramesIdx_(),
    motorsPositionIdx_(),
    motorsVelocityIdx_(),
    positionFieldNames_(),
    velocityFieldNames_(),
    accelerationFieldNames_(),
    motorTorqueFieldNames_(),
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
                               std::vector<std::string> const & motorsNames,
                               bool                     const & hasFreeflyer)
    {
        result_t returnCode = result_t::SUCCESS;

        // Remove all sensors, if any
        sensorsGroupHolder_.clear();
        sensorsDataHolder_.clear();

        // Initialize the URDF model
        contactFramesNames_ = contactFramesNames;
        motorsNames_ = motorsNames;
        returnCode = loadUrdfModel(urdfPath, hasFreeflyer);

        /* Generate the fieldname of the elements of the vectorial representation
           of the configuration, velocity, acceleration and motor torques. */
        positionFieldNames_.clear();
        positionFieldNames_.resize(pncModel_.nq);
        velocityFieldNames_.clear();
        velocityFieldNames_.resize(pncModel_.nv);
        accelerationFieldNames_.clear();
        accelerationFieldNames_.resize(pncModel_.nv);
        std::vector<std::string> jointNames = removeFieldnamesSuffix(pncModel_.names, "Joint");
        for (int32_t jointId=0; jointId<pncModel_.njoints; ++jointId)
        {
            int32_t idx_q = pncModel_.joints[jointId].idx_q();

            if (idx_q >= 0) // Otherwise the joint is not part of the vectorial representation
            {
                int32_t idx_v = pncModel_.joints[jointId].idx_v();

                joint_t jointType;
                std::string jointPrefix;
                if (returnCode == result_t::SUCCESS)
                {
                    returnCode = getJointTypeFromId(*this, jointId, jointType);
                }
                if (returnCode == result_t::SUCCESS)
                {
                    if (jointType == joint_t::FREE)
                    {
                        jointPrefix = FREE_FLYER_PREFIX_BASE_NAME;
                        jointNames[jointId] = ""; // Discard the joint name for FREE joint type since it is unique if any
                    }
                    else
                    {
                        jointPrefix = JOINT_PREFIX_BASE;
                    }

                    returnCode = getJointTypeFromId(*this, jointId, jointType);
                }

                std::vector<std::string> jointTypePositionSuffixes;
                std::vector<std::string> jointPositionFieldnames;
                if (returnCode == result_t::SUCCESS)
                {
                    returnCode = getJointTypePositionSuffixes(jointType, jointTypePositionSuffixes);
                }
                if (returnCode == result_t::SUCCESS)
                {
                    for (std::string const & suffix : jointTypePositionSuffixes)
                    {
                        jointPositionFieldnames.emplace_back(jointPrefix + "Position" + jointNames[jointId] + suffix);
                    }
                }
                if (returnCode == result_t::SUCCESS)
                {
                    std::copy(jointPositionFieldnames.begin(),
                              jointPositionFieldnames.end(),
                              positionFieldNames_.begin() + idx_q);
                }

                std::vector<std::string> jointTypeVelocitySuffixes;
                std::vector<std::string> jointVelocityFieldnames;
                std::vector<std::string> jointAccelerationFieldnames;
                if (returnCode == result_t::SUCCESS)
                {
                    returnCode = getJointTypeVelocitySuffixes(jointType, jointTypeVelocitySuffixes);
                }
                if (returnCode == result_t::SUCCESS)
                {
                    for (std::string const & suffix : jointTypeVelocitySuffixes)
                    {
                        jointVelocityFieldnames.emplace_back(jointPrefix + "Velocity" + jointNames[jointId] + suffix);
                        jointAccelerationFieldnames.emplace_back(jointPrefix + "Acceleration" + jointNames[jointId] + suffix);
                    }
                }
                if (returnCode == result_t::SUCCESS)
                {
                    std::copy(jointVelocityFieldnames.begin(),
                              jointVelocityFieldnames.end(),
                              velocityFieldNames_.begin() + idx_v);
                    std::copy(jointAccelerationFieldnames.begin(),
                              jointAccelerationFieldnames.end(),
                              accelerationFieldNames_.begin() + idx_v);
                }
            }
        }

        motorTorqueFieldNames_.clear();
        for (std::string const & jointName : motorsNames_)
        {
            motorTorqueFieldNames_.emplace_back(JOINT_PREFIX_BASE + "Torque" + jointName);
        }

        // Initialize pinocchio data buffer
        if (returnCode == result_t::SUCCESS)
        {
            pncData_ = pinocchio::Data(pncModel_);
        }

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
            returnCode = getJointsIdx(motorsNames_, motorsPositionIdx_, motorsVelocityIdx_);
        }

        return returnCode;
    }

    void Model::reset(bool const & resetTelemetry)
    {
        // Reset the sensors
        for (sensorsGroupHolder_t::value_type & sensorGroup : sensorsGroupHolder_)
        {
            for (sensorsHolder_t::value_type & sensor : sensorGroup.second)
            {
                sensor.second->reset(resetTelemetry);
            }
        }

        // Reset the telemetry state if needed
        if (resetTelemetry)
        {
            isTelemetryConfigured_ = false;
        }
    }

    result_t Model::configureTelemetry(std::shared_ptr<TelemetryData> const & telemetryData)
    {
        telemetryData_ = std::shared_ptr<TelemetryData>(telemetryData);

        return result_t::SUCCESS;
    }

    result_t Model::removeSensor(std::string const & sensorType,
                                 std::string const & sensorName)
    {
        result_t returnCode = result_t::SUCCESS;

        if (!getIsInitialized())
        {
            std::cout << "Error - Model::removeSensor - Model not initialized." << std::endl;
            return result_t::ERROR_INIT_FAILED;
        }

        sensorsGroupHolder_t::iterator sensorGroupIt;
        if (returnCode == result_t::SUCCESS)
        {
            sensorGroupIt = sensorsGroupHolder_.find(sensorType);
            if (sensorGroupIt == sensorsGroupHolder_.end())
            {
                std::cout << "Error - Model::removeSensor - This type of sensor does not exist." << std::endl;
                returnCode = result_t::ERROR_BAD_INPUT;
            }
        }

        sensorsHolder_t::iterator sensorIt;
        if (returnCode == result_t::SUCCESS)
        {
            sensorIt = sensorGroupIt->second.find(sensorName);
            if (sensorIt == sensorGroupIt->second.end())
            {
                std::cout << "Error - Model::removeSensors - No sensor with this type and name exists." << std::endl;
                returnCode = result_t::ERROR_BAD_INPUT;
            }
        }

        if (returnCode == result_t::SUCCESS)
        {
            // Remove the sensor from its group
            sensorGroupIt->second.erase(sensorIt);

            // Remove the sensor group if there is no more sensors left.
            if (sensorGroupIt->second.empty())
            {
                sensorsGroupHolder_.erase(sensorType);
                sensorsDataHolder_.erase(sensorType);
            }
        }

        return returnCode;
    }

    result_t Model::removeSensors(std::string const & sensorType)
    {
        result_t returnCode = result_t::SUCCESS;

        if (!getIsInitialized())
        {
            std::cout << "Error - Model::removeSensors - Model not initialized." << std::endl;
            return result_t::ERROR_INIT_FAILED;
        }

        sensorsGroupHolder_t::iterator sensorGroupIt;
        if (returnCode == result_t::SUCCESS)
        {
            sensorGroupIt = sensorsGroupHolder_.find(sensorType);
            if (sensorGroupIt == sensorsGroupHolder_.end())
            {
                std::cout << "Error - Model::removeSensors - No sensor with this type exists." << std::endl;
                returnCode = result_t::ERROR_BAD_INPUT;
            }
        }

        if (returnCode == result_t::SUCCESS)
        {
            sensorsGroupHolder_.erase(sensorGroupIt);
            sensorsDataHolder_.erase(sensorType);
        }

        return returnCode;
    }

    result_t Model::getSensorsOptions(std::string    const & sensorType,
                                      configHolder_t       & sensorsOptions) const
    {
        result_t returnCode = result_t::SUCCESS;

        if (!getIsInitialized())
        {
            std::cout << "Error - Model::getSensorOptions - Model not initialized." << std::endl;
            return result_t::ERROR_INIT_FAILED;
        }

        sensorsGroupHolder_t::const_iterator sensorGroupIt;
        if (returnCode == result_t::SUCCESS)
        {
            sensorGroupIt = sensorsGroupHolder_.find(sensorType);
            if (sensorGroupIt == sensorsGroupHolder_.end())
            {
                std::cout << "Error - Model::getSensorOptions - This type of sensor does not exist." << std::endl;
                returnCode = result_t::ERROR_BAD_INPUT;
            }
        }

        if (returnCode == result_t::SUCCESS)
        {
            sensorsOptions = configHolder_t();
            for (sensorsHolder_t::value_type const & sensor : sensorGroupIt->second)
            {
                sensorsOptions[sensor.first] = sensor.second->getOptions();
            }
        }

        return returnCode;
    }

    result_t Model::getSensorsOptions(configHolder_t & sensorsOptions) const
    {
        result_t returnCode = result_t::SUCCESS;

        if (!getIsInitialized())
        {
            std::cout << "Error - Model::getSensorsOptions - Model not initialized." << std::endl;
            return result_t::ERROR_INIT_FAILED;
        }

        if (returnCode == result_t::SUCCESS)
        {
            sensorsOptions = configHolder_t();
            for (sensorsGroupHolder_t::value_type const & sensorGroup : sensorsGroupHolder_)
            {
                configHolder_t sensorsGroupOptions;
                for (sensorsHolder_t::value_type const & sensor : sensorGroup.second)
                {
                    sensorsGroupOptions[sensor.first] = sensor.second->getOptions();
                }
                sensorsOptions[sensorGroup.first] = sensorsGroupOptions;
            }
        }

        return returnCode;
    }

    result_t Model::getSensorOptions(std::string    const & sensorType,
                                     std::string    const & sensorName,
                                     configHolder_t       & sensorOptions) const
    {
        result_t returnCode = result_t::SUCCESS;

        if (!getIsInitialized())
        {
            std::cout << "Error - Model::getSensorOptions - Model not initialized." << std::endl;
            return result_t::ERROR_INIT_FAILED;
        }

        sensorsGroupHolder_t::const_iterator sensorGroupIt;
        if (returnCode == result_t::SUCCESS)
        {
            sensorGroupIt = sensorsGroupHolder_.find(sensorType);
            if (sensorGroupIt == sensorsGroupHolder_.end())
            {
                std::cout << "Error - Model::getSensorOptions - This type of sensor does not exist." << std::endl;
                returnCode = result_t::ERROR_BAD_INPUT;
            }
        }

        sensorsHolder_t::const_iterator sensorIt;
        if (returnCode == result_t::SUCCESS)
        {
            sensorIt = sensorGroupIt->second.find(sensorName);
            if (sensorIt == sensorGroupIt->second.end())
            {
                std::cout << "Error - Model::getSensorOptions - No sensor with this type and name exists." << std::endl;
                returnCode = result_t::ERROR_BAD_INPUT;
            }
        }

        if (returnCode == result_t::SUCCESS)
        {
            sensorOptions = sensorIt->second->getOptions();
        }

        return returnCode;
    }

    result_t Model::setSensorOptions(std::string    const & sensorType,
                                     std::string    const & sensorName,
                                     configHolder_t const & sensorOptions)
    {
        result_t returnCode = result_t::SUCCESS;

        if (!getIsInitialized())
        {
            std::cout << "Error - Model::getSensorOptions - Model not initialized." << std::endl;
            return result_t::ERROR_INIT_FAILED;
        }

        sensorsGroupHolder_t::iterator sensorGroupIt;
        if (returnCode == result_t::SUCCESS)
        {
            sensorGroupIt = sensorsGroupHolder_.find(sensorType);
            if (sensorGroupIt == sensorsGroupHolder_.end())
            {
                std::cout << "Error - Model::getSensorOptions - This type of sensor does not exist." << std::endl;
                returnCode = result_t::ERROR_BAD_INPUT;
            }
        }

        sensorsHolder_t::iterator sensorIt;
        if (returnCode == result_t::SUCCESS)
        {
            sensorIt = sensorGroupIt->second.find(sensorName);
            if (sensorIt == sensorGroupIt->second.end())
            {
                std::cout << "Error - Model::getSensorOptions - No sensor with this type and name exists." << std::endl;
                returnCode = result_t::ERROR_BAD_INPUT;
            }
        }

        if (returnCode == result_t::SUCCESS)
        {
            sensorIt->second->setOptions(sensorOptions);
        }

        return returnCode;
    }

    result_t Model::setSensorsOptions(std::string    const & sensorType,
                                      configHolder_t const & sensorsOptions)
    {
        result_t returnCode = result_t::SUCCESS;

        if (!getIsInitialized())
        {
            std::cout << "Error - Model::getSensorOptions - Model not initialized." << std::endl;
            return result_t::ERROR_INIT_FAILED;
        }

        sensorsGroupHolder_t::iterator sensorGroupIt;
        if (returnCode == result_t::SUCCESS)
        {
            sensorGroupIt = sensorsGroupHolder_.find(sensorType);
            if (sensorGroupIt == sensorsGroupHolder_.end())
            {
                std::cout << "Error - Model::getSensorOptions - This type of sensor does not exist." << std::endl;
                returnCode = result_t::ERROR_BAD_INPUT;
            }
        }

        if (returnCode == result_t::SUCCESS)
        {
            for (sensorsHolder_t::value_type const & sensor : sensorGroupIt->second)
            {
                configHolder_t::const_iterator it = sensorsOptions.find(sensor.first);
                if (it != sensorsOptions.end())
                {
                    sensor.second->setOptions(boost::get<configHolder_t>(it->second));
                }
                else
                {
                    sensor.second->setOptionsAll(sensorsOptions);
                    break;
                }
            }
        }

        return returnCode;
    }

    result_t Model::setSensorsOptions(configHolder_t const & sensorsOptions)
    {
        result_t returnCode = result_t::SUCCESS;

        if (!getIsInitialized())
        {
            std::cout << "Error - Model::getSensorOptions - Model not initialized." << std::endl;
            return result_t::ERROR_INIT_FAILED;
        }

        if (returnCode == result_t::SUCCESS)
        {
            for (sensorsGroupHolder_t::value_type const & sensorGroup : sensorsGroupHolder_)
            {
                for (sensorsHolder_t::value_type const & sensor : sensorGroup.second)
                {
                    sensor.second->setOptions(boost::get<configHolder_t>(
                        boost::get<configHolder_t>(sensorsOptions.at(sensorGroup.first)).at(sensor.first))); // TODO: missing check for sensor type and name availability
                }
            }
        }

        return returnCode;
    }

    configHolder_t Model::getOptions(void) const
    {
        return mdlOptionsHolder_;
    }

    result_t Model::setOptions(configHolder_t mdlOptions)
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
                boundsMin = vectorN_t::Zero(motorsPositionIdx_.size());
                boundsMax = vectorN_t::Zero(motorsPositionIdx_.size());
                for (uint32_t i=0; i < motorsPositionIdx_.size(); i++)
                {
                    boundsMin[i] = pncModel_.lowerPositionLimit[motorsPositionIdx_[i]];
                    boundsMax[i] = pncModel_.upperPositionLimit[motorsPositionIdx_[i]];
                }
            }
            else
            {
                if((int32_t) motorsPositionIdx_.size() != boundsMin.size() || (uint32_t) motorsPositionIdx_.size() != boundsMax.size())
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

    bool Model::getHasFreeFlyer(void) const
    {
        return hasFreeflyer_;
    }

    std::map<std::string, std::vector<std::string> > Model::getSensorsNames(void) const
    {
        std::map<std::string, std::vector<std::string> > sensorNames;
        for (sensorsGroupHolder_t::value_type const & sensorGroup : sensorsGroupHolder_)
        {
            for (sensorsHolder_t::value_type const & sensor : sensorGroup.second)
            {
                sensorNames[sensorGroup.first].push_back(sensor.first);
            }
        }
        return sensorNames;
    }

    result_t Model::loadUrdfModel(std::string const & urdfPath,
                                  bool        const & hasFreeflyer)
    {
        result_t returnCode = result_t::SUCCESS;

        if (!std::ifstream(urdfPath.c_str()).good())
        {
            std::cout << "Error - Model::loadUrdfModel - The URDF file does not exist. Impossible to load it." << std::endl;
            returnCode = result_t::ERROR_BAD_INPUT;
        }
        urdfPath_ = urdfPath;
        hasFreeflyer_ = hasFreeflyer;

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
                std::cout << "Error - Model::loadUrdfModel - Something is wrong with the URDF. Impossible to build a model from it." << std::endl;
                returnCode = result_t::ERROR_BAD_INPUT;
            }
        }

        if (returnCode == result_t::SUCCESS)
        {
            setOptions(mdlOptionsHolder_); // Make sure bounds are the right ones
        }

        return returnCode;
    }

    result_t Model::getSensorsData(std::string const & sensorType,
                                   matrixN_t         & data) const
    {
        return sensorsGroupHolder_.at(sensorType).begin()->second->getAll(data);
    }

    result_t Model::getSensorData(std::string const & sensorType,
                                  std::string const & sensorName,
                                  vectorN_t         & data) const
    {
        return sensorsGroupHolder_.at(sensorType).at(sensorName)->get(data);
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

    void Model::updateTelemetry(void)
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

    std::vector<std::string> const & Model::getMotorsName(void) const
    {
        return motorsNames_;
    }

    std::vector<int32_t> const & Model::getMotorsPositionIdx(void) const
    {
        return motorsPositionIdx_;
    }

    std::vector<int32_t> const & Model::getMotorsVelocityIdx(void) const
    {
        return motorsVelocityIdx_;
    }

    std::vector<std::string> const & Model::getPositionFieldNames(void) const
    {
        return positionFieldNames_;
    }

    std::vector<std::string> const & Model::getVelocityFieldNames(void) const
    {
        return velocityFieldNames_;
    }

    std::vector<std::string> const & Model::getAccelerationFieldNames(void) const
    {
        return accelerationFieldNames_;
    }

    std::vector<std::string> const & Model::getMotorTorqueFieldNames(void) const
    {
        return motorTorqueFieldNames_;
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

    result_t Model::getJointIdx(std::string const & motorName,
                                int32_t           & motorPositionIdx,
                                int32_t           & motorVelocityIdx) const
    {
        // It only return the index of the first element if the joint has multiple degrees of freedom

        /* Obtained using the cumulative sum of number of variables for each joint previous
            to the desired one in the kinematic chain but skipping the first joint, which
            is always the "universe" and is irrelevant (enforced by pinocchio itself). */

        result_t returnCode = result_t::SUCCESS;

        if (!pncModel_.existJointName(motorName))
        {
            std::cout << "Error - ExoModel::getFrameIdx - Frame '" << motorName << "' not found in urdf." << std::endl;
            returnCode = result_t::ERROR_BAD_INPUT;
        }

        if (returnCode == result_t::SUCCESS)
        {
            int32_t jointModelIdx = pncModel_.getJointId(motorName);
            motorPositionIdx = 0;
            motorVelocityIdx = 0;
            for (auto jointIt = pncModel_.joints.begin() + 1; jointIt != pncModel_.joints.begin() + jointModelIdx; jointIt++)
            {
                motorPositionIdx += jointIt->nq();
                motorVelocityIdx += jointIt->nv();
            }
        }

        return returnCode;
    }

    result_t Model::getJointsIdx(std::vector<std::string> const & motorsNames,
                                 std::vector<int32_t>           & motorsPositionIdx,
                                 std::vector<int32_t>           & motorsVelocityIdx) const
    {
        result_t returnCode = result_t::SUCCESS;

        motorsPositionIdx.resize(0);
        motorsVelocityIdx.resize(0);
        for (std::string const & name : motorsNames)
        {
            if (returnCode == result_t::SUCCESS)
            {
                int32_t positionIdx;
                int32_t velocityIdx;
                returnCode = getJointIdx(name, positionIdx, velocityIdx);
                motorsPositionIdx.push_back(positionIdx);
                motorsVelocityIdx.push_back(velocityIdx);
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
