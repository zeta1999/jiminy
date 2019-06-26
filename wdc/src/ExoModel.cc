#include "exo_simu/core/Sensor.h"
#include "exo_simu/wdc/ExoModel.h"

namespace exo_simu
{
    ExoModel::ExoModel(void) :
    Model(),
    exoMdlOptions_(nullptr)
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

        // The contact frames are hard-coded so far
        contactFramesNames_ = std::vector<std::string>({std::string("LeftExternalToe"),
                                                        std::string("LeftInternalToe"),
                                                        std::string("LeftExternalHeel"),
                                                        std::string("LeftInternalHeel"),
                                                        std::string("RightInternalToe"),
                                                        std::string("RightExternalToe"),
                                                        std::string("RightInternalHeel"),
                                                        std::string("RightExternalHeel")});

        // Initialize base Model (cannot use custom bounds so far)
        configHolder_t & jointConfig = boost::get<configHolder_t>(mdlOptionsHolder_.at("joints"));
        configHolder_t jointConfigOld = jointConfig;
        boost::get<bool>(jointConfig.at("boundsFromUrdf")) = true;
        returnCode = Model::initialize(urdfPath, contactFramesNames_, jointsNames_); // The joint names are unknown at this point
        jointConfig = jointConfigOld;
        isInitialized_ = false; // ExoModel is not initialized so far

        std::vector<int32_t> imuFramesIdx;
        std::vector<std::string> imuFramesNames;
        if (returnCode == result_t::SUCCESS)
        {
            // The joint names are obtained by removing the universe and root joint from the joint list
            jointsNames_.assign(pncModel_.names.begin() + 2, pncModel_.names.end());

            // Discard the toes since they are not actuated nor physically meaningful
            jointsNames_.erase(
                std::remove_if(
                    jointsNames_.begin(), 
                    jointsNames_.end(),
                    [](std::string const & joint) -> bool
                    {
                        return joint.find("Toe") != std::string::npos;
                    }
                ), 
                jointsNames_.end()
            );

            /* Update the joint names manually to avoid calling back Model::initialize
               (It cannot throw an error since the names are extracted dynamically from the model) */
            getJointsIdx(jointsNames_, jointsPositionIdx_, jointsVelocityIdx_);

            // The names of the frames of the IMUs end with IMU
            for (int32_t i = 0; i < pncModel_.nframes; i++)
            {
                std::string frameNameCurrent = pncModel_.frames[i].name;
                if (frameNameCurrent.substr(frameNameCurrent.length() - 3) == "IMU")
                {
                    imuFramesNames.push_back(frameNameCurrent);
                }
            }
            getFramesIdx(imuFramesNames, imuFramesIdx); // It cannot throw an error
        }

        // Define the common IMU sensor parameter(s)
        std::string imuBaseName = "IMUSensor";

        // Add the IMU sensors
        for(uint32_t i = 0; i < imuFramesNames.size(); i++)
        {
            if (returnCode == result_t::SUCCESS)
            {
                // Create a new IMU sensor
                std::string imuName = imuBaseName + "_" + imuFramesNames[i];
                ImuSensor imu(imuName);
                configHolder_t imuOptions = imu.getOptions();
                imu.setOptions(imuOptions);

                // Config the IMU
                imu.initialize(imuFramesIdx[i]);

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
        
         // Update the bounds if necessary and set the initialization flag
        if (returnCode == result_t::SUCCESS)
        {
            isInitialized_ = true;
            returnCode = setOptions(mdlOptionsHolder_);
        }

        return returnCode;
    }

    result_t ExoModel::setOptions(configHolder_t const & mdlOptions)
    {
        result_t returnCode = result_t::SUCCESS;

        returnCode = Model::setOptions(mdlOptions);
        if (returnCode == result_t::SUCCESS)
        {
            exoMdlOptions_ = std::make_shared<exoModelOptions_t const>(mdlOptionsHolder_);
        }

        return returnCode;
    }
}