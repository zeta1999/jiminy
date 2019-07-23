#include "exo_simu/core/Sensor.h"
#include "exo_simu/wdc/ExoModel.h"


namespace exo_simu
{
    ExoModel::ExoModel(void) :
    Model(),
    exoMdlOptions_(nullptr),
    toesNames_(),
    toesVelocityIdx_()
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
        returnCode = Model::initialize(urdfPath, contactFramesNames_, motorsNames_); // The joint names are unknown at this point
        boost::get<configHolder_t>(mdlOptionsHolder_.at("joints")) = jointConfigOld; // Be careful, the reference jointConfig may become invalid after calling 'initialize' method.
        isInitialized_ = true;

        if (returnCode == result_t::SUCCESS)
        {
            // The joint names are obtained by removing the root joint from the joint list
            motorsNames_ = getRigidJointsNames();
            motorsNames_.erase(motorsNames_.begin());
            toesNames_ = motorsNames_;

            // Separate the toes froms the other joints since they are not actuated nor physically meaningful
            auto detectToeFct = [](std::string const & joint) -> bool
                                {
                                    return joint.find("Toe") != std::string::npos;
                                };
            motorsNames_.erase(std::remove_if(motorsNames_.begin(),
                                              motorsNames_.end(),
                                              detectToeFct),
                               motorsNames_.end());
            toesNames_.erase(std::remove_if(toesNames_.begin(),
                                            toesNames_.end(),
                                            not_f(detectToeFct)),
                             toesNames_.end());

            getJointsPositionIdx(pncModel_, motorsNames_, motorsPositionIdx_, true);
            getJointsVelocityIdx(pncModel_, motorsNames_, motorsVelocityIdx_, true);
            getJointsVelocityIdx(pncModel_, toesNames_, toesVelocityIdx_, true);

            // The names of the frames of the IMUs end with IMU
            std::vector<std::string> imuFramesNames;
            for (int32_t i = 0; i < pncModel_.nframes; i++)
            {
                std::string frameNameCurrent = pncModel_.frames[i].name;
                if (frameNameCurrent.substr(frameNameCurrent.length() - 3) == "IMU")
                {
                    imuFramesNames.push_back(frameNameCurrent);
                }
            }
            std::vector<int32_t> imuFramesIdx;
            getFramesIdx(pncModel_, imuFramesNames, imuFramesIdx); // It cannot throw an error

            // ********** Add the IMU sensors **********
            for(uint32_t i = 0; i < imuFramesNames.size(); i++)
            {
                std::shared_ptr<ImuSensor> imuSensor;
                std::string imuName = imuFramesNames[i];

                if (returnCode == result_t::SUCCESS)
                {
                    // Create a new IMU sensor
                    returnCode = addSensor(imuName, imuSensor);
                }

                if (returnCode == result_t::SUCCESS)
                {
                    // Configure the sensor
                    imuSensor->initialize(imuFramesNames[i]);
                }
            }

            // ********** Add the force sensors **********
            for (uint32_t i = 0; i<contactFramesNames_.size(); i++)
            {
                std::shared_ptr<ForceSensor> forceSensor;
                std::string forceName = contactFramesNames_[i];

                if (returnCode == result_t::SUCCESS)
                {
                    // Create a new force sensor
                    returnCode = addSensor(forceName, forceSensor);
                }

                if (returnCode == result_t::SUCCESS)
                {
                    // Configure the sensor
                    forceSensor->initialize(contactFramesNames_[i]);
                }
            }

            // ********** Add the encoder sensors **********
            for (uint32_t i = 0; i<motorsNames_.size(); i++)
            {
                std::shared_ptr<EncoderSensor> encoderSensor;
                std::string encoderName = motorsNames_[i];

                if (returnCode == result_t::SUCCESS)
                {
                    // Create a new encoder sensor
                    returnCode = addSensor(encoderName, encoderSensor);
                }

                if (returnCode == result_t::SUCCESS)
                {
                    // Configure the sensor
                    encoderSensor->initialize(motorsNames_[i]);
                }
            }
        }

        // Update the bounds if necessary
        if (returnCode == result_t::SUCCESS)
        {
            returnCode = setOptions(mdlOptionsHolder_);
        }

        // Set the initialization flag
        if (returnCode != result_t::SUCCESS)
        {
            isInitialized_ = false;
        }

        return returnCode;
    }

    void ExoModel::reset(void)
    {
        Model::reset();

        getJointsVelocityIdx(pncModel_, toesNames_, toesVelocityIdx_, true);
    }

    result_t ExoModel::configureTelemetry(std::shared_ptr<TelemetryData> const & telemetryData)
    {
        result_t returnCode = result_t::SUCCESS;

        bool isTelemetryConfigured = isTelemetryConfigured_; // Backup the original state since it will be updated
        returnCode = Model::configureTelemetry(telemetryData);

        if (returnCode == result_t::SUCCESS)
        {
            if (!isTelemetryConfigured)
            {
                for (sensorsGroupHolder_t::value_type const & sensorGroup : sensorsGroupHolder_)
                {
                    for (sensorsHolder_t::value_type const & sensor : sensorGroup.second)
                    {
                        if (returnCode == result_t::SUCCESS)
                        {
                            if (sensorGroup.first == ImuSensor::type_)
                            {
                                if (exoMdlOptions_->telemetry.enableImuSensors)
                                {
                                    returnCode = sensor.second->configureTelemetry(telemetryData_);
                                }
                            }
                            else if (sensorGroup.first == ForceSensor::type_)
                            {
                                if (exoMdlOptions_->telemetry.enableForceSensors)
                                {
                                    returnCode = sensor.second->configureTelemetry(telemetryData_);
                                }
                            }
                            else
                            {
                                if (exoMdlOptions_->telemetry.enableEncoderSensors)
                                {
                                    returnCode = sensor.second->configureTelemetry(telemetryData_);
                                }
                            }
                        }
                    }
                }
            }
        }

        if (returnCode != result_t::SUCCESS)
        {
            isTelemetryConfigured_ = false;
        }

        return returnCode;
    }

    result_t ExoModel::setOptions(configHolder_t const & mdlOptions)
    {
        result_t returnCode = result_t::SUCCESS;

        returnCode = Model::setOptions(mdlOptions);
        if (returnCode == result_t::SUCCESS)
        {
            exoMdlOptions_ = std::make_unique<exoModelOptions_t const>(mdlOptionsHolder_);
        }

        return returnCode;
    }

    std::vector<int32_t> const & ExoModel::getToesVelocityIdx(void) const
    {
        return toesVelocityIdx_;
    }
}