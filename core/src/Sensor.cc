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
            if(sensorOptions_->rawData)
            {
                pinocchio::Motion const gyroIMU = pinocchio::getFrameVelocity(model_->pncModel_, model_->pncData_, frameIdx_);
                Eigen::Vector3d const omega = gyroIMU.angular();
                data().head(3) = omega;
                pinocchio::Motion const acceleroIMU = pinocchio::getFrameAcceleration(model_->pncModel_, model_->pncData_, frameIdx_);
                Eigen::Vector3d const accel = acceleroIMU.linear();
                data().tail(3) = accel;
            }
            else
            {
                Eigen::Matrix3d const & rot = model_->pncData_.oMf[frameIdx_].rotation();
                quaternion_t const quat(rot); // Convert a rotation matrix to a quaternion
                data().head(4) = quat.coeffs(); // (x,y,z,w)
                pinocchio::Motion const gyroIMU = pinocchio::getFrameVelocity(model_->pncModel_, model_->pncData_, frameIdx_);
                Eigen::Vector3d const omega = rot * gyroIMU.angular(); // Get angular velocity in world frame
                data().tail(3) = omega;
            }
        }

        return returnCode;
    }

    // ===================== ForceSensor =========================

    template<>
    std::string const AbstractSensorTpl<ForceSensor>::type_("ForceSensor");
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
            std::vector<int32_t> const & contactFramesIdx = model_->getContactFramesIdx();
            std::vector<int32_t>::const_iterator it = std::find(contactFramesIdx.begin(), contactFramesIdx.end(), frameIdx_);
            data() = model_->contactForces_[std::distance(contactFramesIdx.begin(), it)].linear();
        }

        return returnCode;
    }

    // ===================== EncoderSensor =========================

    template<>
    std::string const AbstractSensorTpl<EncoderSensor>::type_("EncoderSensor");
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
            if(sensorOptions_->rawData)
            {
                data() = q.segment<1>(jointPositionIdx_);
            }
            else
            {
                data().head(1) = q.segment<1>(jointPositionIdx_);
                data().tail(1) = v.segment<1>(jointVelocityIdx_);
            }
        }

        return returnCode;
    }
}