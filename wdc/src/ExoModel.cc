#include "exo_simu/core/Sensor.h"
#include "exo_simu/wdc/ExoModel.h"

namespace exo_simu
{
    ExoModel::ExoModel(void) :
    Model(),
    exoMdlOptions_(nullptr),
    contactFramesNames_{std::string("LeftExternalToe"),
                        std::string("LeftInternalToe"),
                        std::string("LeftExternalHeel"),
                        std::string("LeftInternalHeel"),
                        std::string("RightInternalToe"),
                        std::string("RightExternalToe"),
                        std::string("RightInternalHeel"),
                        std::string("RightExternalHeel")},
    imuFramesNames_(),
    jointsNames_(),
    imuFramesIdx_()
    {
        /* Model constructor calls the base implementations of the virtual methods since the derived 
           class is not available at this point. Thus it must be called explicitly in the constructor. */
        setOptions(getDefaultOptions());
    }

    ExoModel::~ExoModel(void)
    {
        // Empty.
    }

    Model* ExoModel::clone(void)
    {
        return new ExoModel(*this);
    }

    result_t ExoModel::initialize(std::string const & urdfPath)
    {
        result_t returnCode = result_t::SUCCESS;

        // Initialize base Model (cannot use custom bounds so far)
        configHolder_t & jointConfig = boost::get<configHolder_t>(mdlOptionsHolder_.at("joints"));
        configHolder_t jointConfigOld = jointConfig;
        boost::get<bool>(jointConfig.at("boundsFromUrdf")) = true;
        returnCode = Model::initialize(urdfPath, contactFramesIdx_, jointsPositionIdx_, jointsVelocityIdx_);
        jointConfig = jointConfigOld;
        isInitialized_ = false; // ExoModel is not initialized so far

        // Extract some joint and frame names and indices
        if (returnCode == result_t::SUCCESS)
        {
            returnCode = getFramesIdx(contactFramesNames_, contactFramesIdx_);
        }
        if (returnCode == result_t::SUCCESS)
        {
            // Just remove the universe and root joint from the joint list
            jointsNames_.assign(pncModel_.names.begin() + 2, pncModel_.names.end());

            // The names of the frames of the IMUs end with IMU
            imuFramesNames_.clear();
            for (int32_t i = 0; i < pncModel_.nframes; i++)
            {
                std::string frameNameCurrent = pncModel_.frames[i].name;
                if (frameNameCurrent.substr(frameNameCurrent.length() - 3) == "IMU")
                {
                    imuFramesNames_.push_back(frameNameCurrent);
                }
            }

            // Impossible to throw errors since the names are extracted from the model dynamically
            getJointsIdx(jointsNames_, jointsPositionIdx_, jointsVelocityIdx_);
            getFramesIdx(imuFramesNames_, imuFramesIdx_);
            
        }

        // Define the common IMU sensor parameter(s)
        std::string imuBaseName = "IMUSensor";

        // Add the IMU sensors
        for(uint32_t i = 0; i < imuFramesNames_.size(); i++)
        {
            if (returnCode == result_t::SUCCESS)
            {
                // Create a new IMU sensor
                std::string imuName = imuBaseName + "_" + imuFramesNames_[i];
                ImuSensor imu(imuName);
                configHolder_t imuOptions = imu.getOptions();
                imu.setOptions(imuOptions);

                // Config the IMU
                imu.initialize(imuFramesIdx_[i]);

                // Add the IMU to the model
                returnCode = addSensor(imuBaseName, &imu);
            }
        }
        
        // Define the common IMU sensor parameter(s)
        std::string forceBaseName = "ForceSensor";

        // Add the force sensors
        for (uint32_t i = 0; i<contactFramesNames_.size(); i++)
        {
            if (returnCode == result_t::SUCCESS)
            {
                // Create a new force sensor
                std::string forceName = forceBaseName + "_" + contactFramesNames_[i];
                ForceSensor forceSensor(forceName);
                configHolder_t forceOptions = forceSensor.getOptions();
                forceSensor.setOptions(forceOptions);

                // Config the force
                forceSensor.initialize(contactFramesIdx_[i]);

                // Add the force to the model
                returnCode = addSensor(forceBaseName, &forceSensor);
            }
        }
        
        if (returnCode == result_t::SUCCESS)
        {
            isInitialized_ = true;
            returnCode = setOptions(mdlOptionsHolder_); // Update the bounds if necessary
        }

        return returnCode;
    }

    result_t ExoModel::setOptions(configHolder_t const & mdlOptions)
    {
        result_t returnCode = result_t::SUCCESS;

        returnCode = Model::setOptions(mdlOptions);
        if (returnCode == result_t::SUCCESS)
        {
            exoMdlOptions_ = std::shared_ptr<exoModelOptions_t const>(new exoModelOptions_t(mdlOptionsHolder_));
        }

        return returnCode;
    }

    result_t ExoModel::getFrameIdx(std::string const & frameName, 
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

    result_t ExoModel::getFramesIdx(std::vector<std::string> const & framesNames, 
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

    result_t ExoModel::getJointIdx(std::string const & jointName, 
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

    result_t ExoModel::getJointsIdx(std::vector<std::string> const & jointsNames, 
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
}