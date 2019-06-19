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
    imuFramesIdx_(),
    jointsIdx_()
    {
        /* Model constructor calls the base implementations of the virtual methods since the derived 
           class is not available at this point. Thus it must be called explicitly in the constructor. */
        setOptions(getDefaultOptions());
    }

    ExoModel::~ExoModel(void)
    {
        // Empty.
    }


    result_t ExoModel::initialize(std::string const & urdfPath)
    {
        result_t returnCode = result_t::SUCCESS;

        // Initialize base Model
        returnCode = setUrdfPath(urdfPath);
        if (returnCode == result_t::SUCCESS)
        {
            returnCode = Model::initialize(urdfPath, contactFramesIdx_);
        }
        isInitialized_ = false; // ExoModel is not initialized so far

        // Extract some joint and frame names and indices
        if (returnCode == result_t::SUCCESS)
        {
            returnCode = getFramesIdx(contactFramesNames_, contactFramesIdx_);
        }
        if (returnCode == result_t::SUCCESS)
        {
            // The names of the frames of the IMUs end with IMU
            imuFramesNames_.clear()
            for (uint32_t i = 0; i < pncModel_.nframes; i++)
            {
                std::string frameNameCurrent = pncModel_.frames[i].name;
                if (frameNameCurrent.substr(frameNameCurrent.length() - 3) == "IMU")
                {
                    imuFramesNames_.push_back(frameNameCurrent);
                }
            }
            // Just remove the universe and root joint from the joint list
            jointsNames_.assign(pncModel_l.names.begin() + 2, pncModel_l.names.end());

            // Impossible to throw errors since the names are extracted from the model dynamically
            getFramesIdx(imuFramesNames_, imuFramesIdx_);
            getJointsIdx(jointsNames_, jointsIdx_);
        }

        // Define the common IMU sensor parameter(s)
        std::string imuBaseName = "IMU";
        std::vector<std::string> imuHeaderSuffixes = {std::string("QUAT_W"), 
                                                      std::string("QUAT_X"), 
                                                      std::string("QUAT_Y"), 
                                                      std::string("QUAT_Z"), 
                                                      std::string("W_X"), 
                                                      std::string("W_Y"), 
                                                      std::string("W_Z")};

        // Add the IMU sensors
        for(uint32_t i = 0; i < imuFramesNames_.size(); i++)
        {
            if (returnCode == result_t::SUCCESS)
            {
                // Create a new IMU sensor
                std::string imuName = imuBaseName + "_" + imuFramesNames_[i];
                ImuSensor imu(imuName, imuHeaderSuffixes);
                configHolder_t imuOptions = imu.getOptions();
                imuOptions["isLoggerEnable"] = true;
                imu.setOptions(imuOptions);

                // Config the IMU
                imu.initialize(imuFramesIdx_[i]);

                // Add the IMU to the model
                returnCode = addSensor(&imu);
            }
        }
        
        // Define the common IMU sensor parameter(s)
        std::string forceBaseName = "Force";
        std::vector<std::string> forceHeaderSuffixes = {std::string("X"), 
                                                        std::string("Y"), 
                                                        std::string("Z")};

        // Add the force sensors
        for (uint32_t i = 0; i<contactFramesNames_.size(); i++)
        {
            if (returnCode == result_t::SUCCESS)
            {
                // Create a new force sensor
                std::string forceName = forceBaseName + "_" + contactFramesNames_[i];
                ForceSensor forceSensor(forceName, forceHeaderSuffixes);
                configHolder_t forceOptions = forceSensor.getOptions();
                forceOptions["isLoggerEnable"] = true;
                forceSensor.setOptions(forceOptions);

                // Config the force
                forceSensor.initialize(contactFramesIdx_[i]);

                // Add the force to the model
                returnCode = addSensor(&forceSensor);
            }
        }
        
        if (returnCode == result_t::SUCCESS)
        {
            isInitialized_ = true;
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
}