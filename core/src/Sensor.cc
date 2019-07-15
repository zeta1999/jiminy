#include <algorithm>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/frames.hpp"

#include "exo_simu/core/Utilities.h"
#include "exo_simu/core/Model.h"
#include "exo_simu/core/Sensor.h"


namespace exo_simu
{
    // ===================== ImuSensor =========================

    template<>
    std::string const AbstractSensorTpl<ImuSensor>::type_("ImuSensor");
    template<>
    uint32_t const AbstractSensorTpl<ImuSensor>::sizeOf_(7);
    template<>
    std::vector<std::string> const AbstractSensorTpl<ImuSensor>::fieldNamesPreProcess_({"w_x", "w_y", "w_z", "a_x", "a_y", "a_z"});
    template<>
    std::vector<std::string> const AbstractSensorTpl<ImuSensor>::fieldNamesPostProcess_({"quat_x", "quat_y", "quat_z", "quat_w", "w_x", "w_y", "w_z"});

    ImuSensor::ImuSensor(Model                               const & model,
                         std::shared_ptr<SensorDataHolder_t> const & dataHolder,
                         std::string                         const & name) :
    AbstractSensorTpl(model, dataHolder, name),
    frameIdx_()
    {
        // Empty.
    }

    ImuSensor::~ImuSensor(void)
    {
        // Empty.
    }

    void ImuSensor::initialize(int32_t const & frameIdx)
    {
        frameIdx_ = frameIdx;
        isInitialized_ = true;
    }

    int32_t ImuSensor::getFrameIdx(void) const
    {
        return frameIdx_;
    }

    result_t ImuSensor::set(float64_t const & t,
                            vectorN_t const & q,
                            vectorN_t const & v,
                            vectorN_t const & a,
                            vectorN_t const & u)
    {
        result_t returnCode = result_t::SUCCESS;

        if (!isInitialized_)
        {
            std::cout << "Error - ImuSensor::set - Sensor not initialized. Impossible to set sensor data." << std::endl;
            returnCode = result_t::ERROR_INIT_FAILED;
        }

        if (returnCode == result_t::SUCCESS)
        {
            // Compute the true value
            Eigen::Matrix4d const tformIMU = model_->pncData_.oMf[frameIdx_].toHomogeneousMatrix();
            Eigen::Matrix3d const rotIMU = tformIMU.topLeftCorner<3,3>();
            quaternion_t const quatIMU(rotIMU); // Convert a rotation matrix to a quaternion
            data().head(4) = quatIMU.coeffs(); // (x,y,z,w)
            pinocchio::Motion motionIMU = pinocchio::getFrameVelocity(model_->pncModel_, model_->pncData_, frameIdx_);
            Eigen::Vector3d omegaIMU = motionIMU.angular();
            data().tail(3) = omegaIMU;

            // Add white noise and bias
            if (sensorOptions_->noiseStd.size())
            {
                data() += randVectorNormal(sizeOf_, sensorOptions_->noiseStd);
            }
            if (sensorOptions_->bias.size())
            {
                data() += sensorOptions_->bias;
            }
        }

        return returnCode;
    }

    // ===================== ForceSensor =========================

    template<>
    std::string const AbstractSensorTpl<ForceSensor>::type_("ForceSensor");
    template<>
    uint32_t const AbstractSensorTpl<ForceSensor>::sizeOf_(3);
    template<>
    std::vector<std::string> const AbstractSensorTpl<ForceSensor>::fieldNamesPreProcess_({"F_x", "F_y", "F_z"});
    template<>
    std::vector<std::string> const AbstractSensorTpl<ForceSensor>::fieldNamesPostProcess_({"F_x", "F_y", "F_z"});

    ForceSensor::ForceSensor(Model                               const & model,
                             std::shared_ptr<SensorDataHolder_t> const & dataHolder,
                             std::string                         const & name) :
    AbstractSensorTpl(model, dataHolder, name),
    frameIdx_()
    {
        // Empty.
    }

    ForceSensor::~ForceSensor(void)
    {
        // Empty.
    }

    void ForceSensor::initialize(int32_t const & frameIdx)
    {
        frameIdx_ = frameIdx;
        isInitialized_ = true;
    }

    int32_t ForceSensor::getFrameIdx(void) const
    {
        return frameIdx_;
    }

    result_t ForceSensor::set(float64_t const & t,
                              vectorN_t const & q,
                              vectorN_t const & v,
                              vectorN_t const & a,
                              vectorN_t const & u)
    {
        result_t returnCode = result_t::SUCCESS;

        if (!isInitialized_)
        {
            std::cout << "Error - ForceSensor::set - Sensor not initialized. Impossible to set sensor data." << std::endl;
            returnCode = result_t::ERROR_INIT_FAILED;
        }

        if (returnCode == result_t::SUCCESS)
        {
            // Compute the true value
            std::vector<int32_t> const & contactFramesIdx = model_->getContactFramesIdx();
            std::vector<int32_t>::const_iterator it = std::find(contactFramesIdx.begin(), contactFramesIdx.end(), frameIdx_);
            data() = model_->contactForces_[std::distance(contactFramesIdx.begin(), it)].linear();

            // Add white noise
            if (sensorOptions_->noiseStd.size())
            {
                data() += randVectorNormal(sizeOf_, sensorOptions_->noiseStd);
            }
            if (sensorOptions_->bias.size())
            {
                data() += sensorOptions_->bias;
            }
        }

        return returnCode;
    }

    // ===================== EncoderSensor =========================

    template<>
    std::string const AbstractSensorTpl<EncoderSensor>::type_("EncoderSensor");
    template<>
    uint32_t const AbstractSensorTpl<EncoderSensor>::sizeOf_(2);
    template<>
    std::vector<std::string> const AbstractSensorTpl<EncoderSensor>::fieldNamesPreProcess_({"q"});
    template<>
    std::vector<std::string> const AbstractSensorTpl<EncoderSensor>::fieldNamesPostProcess_({"q", "v"});

    EncoderSensor::EncoderSensor(Model                               const & model,
                                 std::shared_ptr<SensorDataHolder_t> const & dataHolder,
                                 std::string                         const & name) :
    AbstractSensorTpl(model, dataHolder, name),
    jointPositionIdx_(),
    jointVelocityIdx_()
    {
        // Empty.
    }

    EncoderSensor::~EncoderSensor(void)
    {
        // Empty.
    }

    void EncoderSensor::initialize(int32_t const & jointPositionIdx,
                                   int32_t const & jointVelocityIdx)
    {
        jointPositionIdx_ = jointPositionIdx;
        jointVelocityIdx_ = jointVelocityIdx;
        isInitialized_ = true;
    }

    int32_t EncoderSensor::getJointPositionIdx(void) const
    {
        return jointPositionIdx_;
    }

    int32_t EncoderSensor::getJointVelocityIdx(void) const
    {
        return jointVelocityIdx_;
    }

    result_t EncoderSensor::set(float64_t const & t,
                                vectorN_t const & q,
                                vectorN_t const & v,
                                vectorN_t const & a,
                                vectorN_t const & u)
    {
        result_t returnCode = result_t::SUCCESS;

        if (!isInitialized_)
        {
            std::cout << "Error - EncoderSensor::set - Sensor not initialized. Impossible to set sensor data." << std::endl;
            returnCode = result_t::ERROR_INIT_FAILED;
        }

        if (returnCode == result_t::SUCCESS)
        {
            // Compute the true value
            data().head(1) = q.segment<1>(jointPositionIdx_);
            data().tail(1) = v.segment<1>(jointVelocityIdx_);

            // Add white noise
            if (sensorOptions_->noiseStd.size())
            {
                data() += randVectorNormal(sizeOf_, sensorOptions_->noiseStd);
            }
            if (sensorOptions_->bias.size())
            {
                data() += sensorOptions_->bias;
            }
        }

        return returnCode;
    }
}